import asyncio
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import pytest
from team_factory.core import Store, result, clean
from team_factory.adapters import MockAdapter, ClaudeAdapter, DotsAdapter
from team_factory.workspace import Brain, Workspaces, git
from team_factory.runtime import Runtime


def output(action='complete', agent=None):
    return dict(summary='Explicit result', action=action, next_agent=agent, instruction='Continue within goal', tests=[], limitations=[])

@pytest.fixture
def store(tmp_path):
    return Store(tmp_path/'state.db')

def task(store):
    store.create_goal('Feature X', 'a')
    store.mode('running')
    return store.claim({'a','b'})

def test_explicit_state_machine_and_events(store):
    t=task(store)
    store.finish(t['id'],t['invocation'],output(),{'a','b'})
    assert store.snapshot()['goals'][0]['status']=='done'
    with store.tx() as c:
        with pytest.raises(Exception): c.execute('DELETE FROM events')
        with pytest.raises(ValueError): store.transition(c,store.get(c,'tasks',t['id']),'ready')
    with pytest.raises(ValueError): store.finish(t['id'],t['invocation'],output(),{'a'})

def test_dependencies_and_shared_workspace(store):
    g=store.create_goal('Goal','a'); first=store.snapshot()['tasks'][0]
    store.create_task(g['id'],'Next','b',[first['id']])
    store.mode('running');t=store.claim({'a','b'})
    assert store.claim({'a','b'}) is None
    store.finish(t['id'],t['invocation'],output(),{'a','b'})
    assert store.snapshot()['goals'][0]['status']=='active'
    assert store.claim({'a','b'})['assigned_to']=='b'

def test_pause_stop_resume(store):
    store.create_goal('Goal','a'); assert store.claim({'a'}) is None
    for mode in ('stopped','paused'):
        store.mode(mode); assert store.claim({'a'}) is None
    store.mode('running'); assert store.claim({'a'})

def test_failure_limits(store):
    t=task(store)
    store.fail(t['id'],t['invocation'],'same failure')
    t=store.claim({'a'});store.fail(t['id'],t['invocation'],'same failure')
    assert store.snapshot()['tasks'][0]['status']=='human_input_required'

def test_retry_limit_distinct_errors(store):
    t=task(store)
    for i in range(3):
        store.fail(t['id'],t['invocation'],f'error {i}')
        if i<2:t=store.claim({'a'})
    assert store.snapshot()['tasks'][0]['retries']==3
    assert store.snapshot()['tasks'][0]['status']=='human_input_required'

def test_recovery_no_replay(store):
    t=task(store);store.recover()
    assert store.claim({'a'}) is None
    assert store.snapshot()['tasks'][0]['orphaned']
    with pytest.raises(ValueError): store.control(t['id'],'retry')

def test_duplicate_dispatch_threads(store):
    store.create_goal('Goal','a');store.mode('running')
    with ThreadPoolExecutor(8) as pool:
        claims=list(pool.map(lambda _:store.claim({'a'}),range(8)))
    assert sum(x is not None for x in claims)==1

def test_concurrency_limit_and_agent_lease(store):
    store.limits['max_parallel_agents']=3
    for agent in ('a','a','b'):store.create_goal('Goal',agent)
    store.mode('running')
    assert store.claim({'a','b'})['assigned_to']=='a'
    assert store.claim({'a','b'})['assigned_to']=='b'
    assert store.claim({'a','b'}) is None

def test_unknown_adapter_escalates(store):
    store.create_goal('Goal','dots');store.mode('running')
    assert store.claim({'a'}) is None
    assert store.snapshot()['tasks'][0]['status']=='human_input_required'

def test_loop_and_age_limits(store):
    store.limits['max_reviews']=1;t=task(store)
    store.finish(t['id'],t['invocation'],output('changes','b'),{'a','b'})
    assert store.snapshot()['tasks'][0]['status']=='human_input_required'
    store.create_goal('Old','b');store.limits['task_age_seconds']=-1
    assert store.claim({'a','b'}) is None

def test_malformed_result():
    for r in ({},output('destroy'),output('handoff','unknown'),output()|{'thinking':'hidden'},output()|{'tests':'bad'}):
        with pytest.raises(ValueError):result(r,{'a'})
    assert 'hidden' not in json.dumps(clean({'thinking':'hidden','summary':'<analysis>hidden</analysis> sk-secret'}))

def test_context_paths_and_budget(tmp_path):
    (tmp_path/'brain').mkdir();(tmp_path/'brain/a.md').write_text('a'*100)
    (tmp_path/'outside.md').write_text('secret')
    (tmp_path/'brain/link.md').symlink_to(tmp_path/'outside.md')
    b=Brain(tmp_path,20)
    assert b.select(['brain/a.md'])[0]['truncated']
    for p in ('../outside.md','outside.md','brain/link.md'):
        with pytest.raises(ValueError):b.select([p])
    with pytest.raises(ValueError):b.select(['brain/a.md']*6)

@pytest.fixture
def repo(tmp_path):
    root=tmp_path/'repo';root.mkdir();git(root,'init');git(root,'config','user.email','test@example.invalid');git(root,'config','user.name','Test')
    (root/'file').write_text('baseline');(root/'brain').mkdir();(root/'brain/HANDOFF.md').write_text('Existing roles unchanged')
    git(root,'add','.');git(root,'commit','-m','baseline')
    return root

def test_dirty_root_preserved_and_conflict(repo,tmp_path):
    (repo/'file').write_text('user dirty')
    w=Workspaces(repo,tmp_path/'state');g=w.prepare({'id':'G-test','workspace':None})
    assert (Path(g['workspace'])/'file').read_text()=='baseline'
    assert (repo/'file').read_text()=='user dirty'
    assert g['source_dirty']
    git(g['workspace'],'checkout','--detach')
    with pytest.raises(RuntimeError):w.evidence(g)

def test_actual_git_conflict(repo,tmp_path):
    w=Workspaces(repo,tmp_path/'state');g=w.prepare({'id':'G-conflict','workspace':None});p=Path(g['workspace'])
    (p/'file').write_text('branch');git(p,'commit','-am','branch')
    (repo/'file').write_text('main');git(repo,'commit','-am','main')
    subprocess.run(['git','-C',str(p),'merge',git(repo,'rev-parse','HEAD')],capture_output=True)
    with pytest.raises(RuntimeError,match='conflicts'):w.evidence(g)

def test_mock_e2e_restart(repo,tmp_path):
    async def run():
        state=tmp_path/'runtime';s=Store(state/'db');a={id:MockAdapter(id,id,[]) for id in ('mock-a','mock-b')}
        r=Runtime(s,a,Workspaces(repo,state),Brain(repo),state);r.acquire()
        s.create_goal('Feature X','mock-a');s.mode('running')
        for i in range(2):
            await r.tick();await asyncio.gather(*list(r.active.values()))
        await r.close()
        # Re-open the DB and runtime, preserving counters and worktree ownership.
        s=Store(state/'db');r=Runtime(s,a,Workspaces(repo,state),Brain(repo),state);r.acquire()
        for i in range(2):
            await r.tick();await asyncio.gather(*list(r.active.values()))
        snap=s.snapshot();assert snap['goals'][0]['status']=='done'
        assert [e['data']['agent'] for e in snap['events'] if e['kind']=='AGENT_STARTED']==['mock-a','mock-b','mock-a','mock-b']
        assert sum(e['kind']=='CHANGES_REQUESTED' for e in snap['events'])==1
        assert snap['tasks'][0]['transitions']==4
        await r.close()
    asyncio.run(run())

def test_claude_protocol_and_dots():
    a=ClaudeAdapter('claude','Claude',['CLAUDE.md'])
    cmd=a.command({'session':'explicit-session','session_agent':'claude'})
    assert '--resume' in cmd and '--continue' not in cmd
    assert '--dangerously-skip-permissions' not in cmd and '--system-prompt' not in cmd
    assert a.parse_result(json.dumps({'structured_output':output()}))['action']=='complete'
    for env in ({'is_error':True},{'permission_denials':[{}]},{'result':'not JSON'}):
        with pytest.raises((ValueError,RuntimeError)):a.parse_result(json.dumps(env))
    assert not DotsAdapter('dots','Dots',[]).capability()['autonomous']

def test_runtime_lock(repo,tmp_path):
    state=tmp_path/'runtime';s=Store(state/'db')
    r=Runtime(s,{},Workspaces(repo,state),Brain(repo),state);r.acquire()
    r2=Runtime(s,{},Workspaces(repo,state),Brain(repo),state)
    with pytest.raises(RuntimeError):r2.acquire()
    asyncio.run(r.close())

def test_sensitive_summaries_are_not_persisted(store):
    synthetic = '''{"password": "synthetic pass phrase"} API_KEY='synthetic key tail' <reasoning>private synthetic explanation</reasoning>'''
    t = task(store)
    store.finish(t['id'], t['invocation'], output() | {'summary': synthetic}, {'a'})
    raw = json.dumps(store.snapshot())
    for marker in ('synthetic pass', 'synthetic key', 'private synthetic'):
        assert marker not in raw

def test_cancelled_dependencies_propagate(store):
    g = store.create_goal('Goal', 'a'); first = store.snapshot()['tasks'][0]
    second = store.create_task(g['id'], 'Depends on first', 'b', [first['id']])
    store.create_task(g['id'], 'Depends on second', 'a', [second['id']])
    store.control(first['id'], 'cancel')
    snap = store.snapshot()
    assert all(t['status'] == 'cancelled' for t in snap['tasks'])
    assert snap['goals'][0]['status'] == 'cancelled'

def test_spawn_callback_failure_cleans_process(monkeypatch, tmp_path):
    class FakeProcess:
        pid = 123456
        returncode = None
        def send_signal(self, sig): self.returncode = -sig
        async def wait(self): return self.returncode
    child = FakeProcess()
    async def spawn(*args, **kwargs): return child
    monkeypatch.setattr(asyncio, 'create_subprocess_exec', spawn)
    a = ClaudeAdapter('claude', 'Claude', [])
    a.executable = '/synthetic/claude'; a.sandbox = '/synthetic/sandbox'
    monkeypatch.setattr(a, 'check_configuration', lambda _: None)
    def fail_persist(pid, session): raise RuntimeError('Injected DB failure after spawn')
    with pytest.raises(RuntimeError, match='Injected DB failure'):
        asyncio.run(a.start_task({'invocation':'fault'}, {'workspace':str(tmp_path)}, '{}', fail_persist))
    assert child.returncode is not None
    assert a.status('fault') == 'idle'

def test_null_pid_reconcile_fails_closed(tmp_path):
    import sys
    state = tmp_path/'state'; s = Store(state/'factory.sqlite3'); t = task(s); s.recover()
    # Models a hard orchestrator crash in the interval after spawn and before PID persistence.
    p = subprocess.run([sys.executable, '-m', 'team_factory', '--state', str(state), '--reconcile', t['id']], capture_output=True, text=True)
    assert p.returncode != 0 and 'No persisted process identity' in p.stderr
    assert s.snapshot()['tasks'][0]['orphaned']
    with pytest.raises(ValueError): s.control(t['id'], 'retry')

def test_hook_configuration_gate(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, 'home', lambda: tmp_path/'home')
    (tmp_path/'.claude').mkdir(); p = tmp_path/'.claude/settings.json'
    p.write_text('{"hooks":{"Stop":[]}}')
    with pytest.raises(RuntimeError, match='security review'):
        ClaudeAdapter('claude','Claude',[]).check_configuration(tmp_path)

async def asgi_request(app, method, path, data=None, headers=None):
    messages = []; sent = False
    async def receive():
        nonlocal sent
        if not sent:
            sent=True
            return {'type':'http.request','body':json.dumps(data).encode() if data is not None else b'', 'more_body':False}
        await asyncio.Event().wait()
    async def send(message): messages.append(message)
    h = {'host':'127.0.0.1:8097'} | (headers or {})
    await app({'type':'http','asgi':{'version':'3.0'},'http_version':'1.1','method':method,
               'path':path,'raw_path':path.encode(),'query_string':b'', 'scheme':'http',
               'server':('127.0.0.1',8097),'client':('127.0.0.1',12345),
               'headers':[(k.lower().encode(),v.encode()) for k,v in h.items()]}, receive, send)
    status = next(m['status'] for m in messages if m['type']=='http.response.start')
    payload = b''.join(m.get('body',b'') for m in messages if m['type']=='http.response.body')
    return status, json.loads(payload)

def test_http_security_and_validation(repo,tmp_path):
    from team_factory.api import create_app
    state=tmp_path/'http';s=Store(state/'db');r=Runtime(s,{'a':MockAdapter('a','A',[])},Workspaces(repo,state),Brain(repo),state)
    app=create_app(r,True)
    async def run():
        status,snapshot=await asgi_request(app,'GET','/api/state');assert status==200
        csrf={'x-factory-csrf':snapshot['csrf']}
        for headers in ({},{'host':'attacker.invalid'},csrf|{'origin':'https://attacker.invalid'},csrf|{'sec-fetch-site':'cross-site'}):
            status,_=await asgi_request(app,'POST','/api/goals',{'title':'X','agent':'a'},headers);assert status==403
        for data in ([], {'title':{},'agent':'a'}, {'title':'X','agent':[]}, {'title':'X','agent':'a','brain_context':[{}]}):
            status,_=await asgi_request(app,'POST','/api/goals',data,csrf);assert status==400
        status,_=await asgi_request(app,'POST','/api/goals',{'title':'X','agent':'a'},csrf);assert status==200
        status,_=await asgi_request(app,'POST','/api/team/paused',{},csrf);assert status==200
        assert s.snapshot()['mode']=='paused'
        assert len(s.snapshot()['goals'])==1
    asyncio.run(run())

def test_claude_timeout_stops_owned_process(monkeypatch,tmp_path):
    class Input:
        def write(self, data): pass
        async def drain(self): pass
        def close(self): pass
    class Output:
        async def read(self, n): await asyncio.Event().wait()
    class Process:
        pid=9999; returncode=None; stdin=Input();stdout=Output()
        def send_signal(self,sig):self.returncode=-sig
        async def wait(self):return self.returncode
    child=Process()
    async def spawn(*args,**kwargs):return child
    monkeypatch.setattr(asyncio,'create_subprocess_exec',spawn)
    a=ClaudeAdapter('a','A',[],timeout=.01);a.executable='/fake';a.sandbox='/fake'
    monkeypatch.setattr(a,'check_configuration',lambda _:None)
    with pytest.raises(asyncio.TimeoutError):
        asyncio.run(a.start_task({'invocation':'timed'}, {'workspace':str(tmp_path)}, '{}', lambda *args:None))
    assert child.returncode is not None and a.status('timed')=='idle'

def test_pause_and_stop_drain_active_handoff(repo,tmp_path):
    async def run():
        state=tmp_path/'state';s=Store(state/'db');adapters={id:MockAdapter(id,id,[]) for id in ('mock-a','mock-b')}
        r=Runtime(s,adapters,Workspaces(repo,state),Brain(repo),state);r.acquire()
        s.create_goal('Goal','mock-a');s.mode('running');await r.tick()
        for mode in ('paused','stopped'):
            s.mode(mode)
            await asyncio.gather(*list(r.active.values()))
            await r.tick()
            assert not r.active
            assert s.snapshot()['tasks'][0]['status']=='ready'
        s.mode('running');await r.tick();await asyncio.gather(*list(r.active.values()))
        assert s.snapshot()['tasks'][0]['transitions']==2
        await r.close()
    asyncio.run(run())

def test_permission_failure_escalates_immediately(store):
    t=task(store);store.fail(t['id'],t['invocation'],'Credentials required',immediate=True)
    assert store.snapshot()['tasks'][0]['status']=='human_input_required'
    assert store.claim({'a'}) is None

def test_mixed_cancelled_and_completed_goal_is_terminal(store):
    g=store.create_goal('Goal','a');first=store.snapshot()['tasks'][0]
    store.create_task(g['id'],'Independent','b')
    store.control(first['id'],'cancel');store.mode('running')
    t=store.claim({'a','b'});store.finish(t['id'],t['invocation'],output(),{'a','b'})
    snap=store.snapshot()
    assert snap['goals'][0]['status']=='cancelled'
    assert len(snap['goals'][0]['result']['tasks_completed'])==1
    assert not any(e['kind']=='GOAL_COMPLETED' for e in snap['events'])

def test_config_directory_and_auth_helper_gate(tmp_path,monkeypatch):
    a=ClaudeAdapter('a','A',[])
    monkeypatch.setenv('CLAUDE_CONFIG_DIR',str(tmp_path/'alternate'))
    with pytest.raises(RuntimeError,match='CLAUDE_CONFIG_DIR'):a.check_configuration(tmp_path)
    monkeypatch.delenv('CLAUDE_CONFIG_DIR')
    monkeypatch.setattr(Path,'home',lambda:tmp_path/'home')
    nested=tmp_path/'nested/.claude';nested.mkdir(parents=True)
    (nested/'settings.json').write_text('{"gcpAuthRefresh":"synthetic executable"}')
    with pytest.raises(RuntimeError,match='security review'):a.check_configuration(tmp_path)

def test_managed_settings_fragments_gate(tmp_path,monkeypatch):
    fragment=tmp_path/'managed.json';fragment.write_text('{"hooks":{"Stop":[]}}')
    original=Path.glob
    def glob(path,pattern):
        if str(path)=='/Library/Application Support/ClaudeCode/managed-settings.d':
            return iter([fragment])
        return original(path,pattern)
    monkeypatch.setattr(Path,'glob',glob)
    monkeypatch.setattr(Path,'home',lambda:tmp_path/'home')
    with pytest.raises(RuntimeError,match='security review'):
        ClaudeAdapter('a','A',[]).check_configuration(tmp_path)

def test_server_restart_with_open_event_stream(repo,tmp_path):
    import socket,sys,time,urllib.request,signal
    with socket.socket() as probe:
        try:probe.bind(('127.0.0.1',0))
        except PermissionError:pytest.skip('Local sockets unavailable in this execution sandbox')
        port=probe.getsockname()[1]
    state=tmp_path/'restart'
    cmd=[sys.executable,'-m','team_factory','--repo',str(repo),'--state',str(state),'--demo','--port',str(port)]
    for cycle in range(2):
        process=subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        stream=None
        try:
            deadline=time.monotonic()+8
            while True:
                try:
                    with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state',timeout=.5) as response:
                        snapshot=json.load(response)
                    break
                except OSError:
                    if time.monotonic()>deadline:raise
                    time.sleep(.1)
            assert snapshot['mode']=='paused' and not snapshot['tasks']
            stream=urllib.request.urlopen(f'http://127.0.0.1:{port}/api/events',timeout=5)
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=7)
            assert process.returncode in (0,-signal.SIGTERM)
        finally:
            if stream:stream.close()
            if process.poll() is None:process.kill();process.wait()
