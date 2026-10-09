import asyncio,json,time
import pytest
from team_factory.worker import Mailbox,ExternalWorkerAdapter
from team_factory.adapters import registry,HumanRequired


def response():
 return dict(summary='Explicit external response',action='complete',next_agent=None,instruction='',tests=[],limitations=[])


def test_worker_attach_identity_and_expiry(tmp_path):
 b=Mailbox(tmp_path,'dots',ttl=1)
 assert not b.online()
 b.attach('actual-session');assert b.online()
 with pytest.raises(ValueError):b.attach('duplicate')
 with pytest.raises(ValueError):b.heartbeat('wrong')
 with b.lock():
  p=b.read('presence.json');p['heartbeat_at']=time.time()-10;b.write('presence.json',p)
 assert not b.online()
 with pytest.raises(ValueError):b.heartbeat('actual-session')


def test_worker_paths(tmp_path):
 for id in ('../escape','a/b',''):
  with pytest.raises(ValueError):Mailbox(tmp_path,id)
 (tmp_path/'workers').symlink_to(tmp_path/'outside')
 with pytest.raises(ValueError):Mailbox(tmp_path,'dots')


def test_duplicate_claim_and_stale_finish(tmp_path):
 b=Mailbox(tmp_path,'dots');b.attach('session')
 offer=dict(worker_id='session',invocation='invoke',lease='fence',claimed=False,closed=False)
 with b.lock():b.write('offer.json',offer)
 assert b.poll('session')['task']['invocation']=='invoke'
 assert b.poll('session')['task'] is None
 with pytest.raises(ValueError):b.complete('session','invoke','old-fence',{})
 with pytest.raises(ValueError):b.detach('session')
 b.complete('session','invoke','fence',response())
 with pytest.raises(ValueError):b.complete('session','invoke','fence',{})


def test_live_worker_adapter_correlates_result(tmp_path):
 async def run():
  a=ExternalWorkerAdapter('dots','Actual Dots',['AGENTS.md'],state=tmp_path)
  assert not a.capability()['autonomous']
  a.mailbox.attach('supervised-thread');assert a.capability()['autonomous']
  job=asyncio.create_task(a.start_task({'id':'T-one','invocation':'I-one'}, {'id':'G-one','title':'User goal'}, json.dumps({'goal':'User goal'}), lambda *args:None))
  await asyncio.sleep(.01)
  offer=a.mailbox.poll('supervised-thread')['task']
  assert offer['authorization']['goal_id']=='G-one'
  assert a.status('I-one')=='running'
  result=response()
  a.mailbox.complete('supervised-thread','I-one',offer['lease'],result)
  assert await job==result
  assert a.status('I-one')=='idle'
  a.mailbox.detach('supervised-thread');assert not a.capability()['autonomous']
 asyncio.run(run())


def test_expired_claim_keeps_uncertain_lease(tmp_path):
 async def run():
  a=ExternalWorkerAdapter('dots','Dots',[],state=tmp_path)
  a.mailbox.attach('session')
  job=asyncio.create_task(a.start_task({'id':'T-one','invocation':'I-one'},{'id':'G-one','title':'Goal'},'{}',lambda *args:None))
  await asyncio.sleep(.01);offer=a.mailbox.poll('session')['task']
  with a.mailbox.lock():
   p=a.mailbox.read('presence.json');p['heartbeat_at']=0;a.mailbox.write('presence.json',p)
  with pytest.raises(HumanRequired):await job
  assert a.status('I-one')=='running'
  with pytest.raises(ValueError):a.mailbox.complete('session','I-one',offer['lease'],{})
  with pytest.raises(ValueError):a.mailbox.attach('replacement')
 asyncio.run(run())


def test_unclaimed_offer_closes_safely(tmp_path):
 async def run():
  a=ExternalWorkerAdapter('dots','Dots',[],state=tmp_path);a.timeout=.01
  a.mailbox.attach('session')
  with pytest.raises(HumanRequired):await a.start_task({'id':'T-one','invocation':'I-one'},{'id':'G-one','title':'Goal'},'{}',lambda *args:None)
  assert a.status('I-one')=='idle'
  assert a.mailbox.poll('session')['task'] is None
 asyncio.run(run())

def test_malformed_result_does_not_release_claim(tmp_path):
 b=Mailbox(tmp_path,'dots');b.attach('session')
 with b.lock():b.write('offer.json',dict(worker_id='session',invocation='invoke',lease='fence',claimed=False,closed=False))
 b.poll('session')
 with pytest.raises(ValueError):b.complete('session','invoke','fence',{'thinking':'private'})
 assert not b.read('result.json')
 assert not b.read('offer.json')['closed']


def test_runtime_lost_worker_is_orphaned(tmp_path):
 from team_factory.core import Store
 from team_factory.runtime import Runtime
 from team_factory.workspace import Brain
 class Workspace:
  def prepare(self,g):return g|dict(workspace=str(tmp_path),branch='test',base_commit='test',baseline='test')
 async def run():
  state=tmp_path/'state';store=Store(state/'db');adapter=ExternalWorkerAdapter('dots','Dots',[],state=state)
  adapter.mailbox.attach('actual-thread')
  runtime=Runtime(store,{'dots':adapter},Workspace(),Brain(tmp_path),state);runtime.acquire()
  store.create_goal('Transport check','dots');store.mode('running');await runtime.tick()
  while not adapter.mailbox.read('offer.json'):await asyncio.sleep(.01)
  adapter.mailbox.poll('actual-thread')
  with adapter.mailbox.lock():
   p=adapter.mailbox.read('presence.json');p['heartbeat_at']=0;adapter.mailbox.write('presence.json',p)
  await asyncio.gather(*runtime.active.values())
  t=store.snapshot()['tasks'][0]
  assert t['status']=='human_input_required' and t['orphaned']
  assert t['retries']==0
  with pytest.raises(ValueError):store.control(t['id'],'retry')
  assert store.claim({'dots'}) is None
  await runtime.close()
 asyncio.run(run())

def test_instruction_read_approval_is_exact_and_one_command(tmp_path):
 from team_factory.adapters import ClaudeAdapter
 (tmp_path/'brain').mkdir()
 for name in ('AGENTS.md','CLAUDE.md','brain/HANDOFF.md'):(tmp_path/name).write_text('Existing instructions')
 a=ClaudeAdapter('claude','Claude',[])
 paths=a.approve_instruction_reads_once('T-smoke',tmp_path)
 assert '--allowedTools' not in a.command({'id':'T-other'})
 cmd=a.command({'id':'T-smoke'})
 i=cmd.index('--allowedTools')
 assert cmd[i+1:i+4]==['Read(/'+str(p)+')' for p in paths]
 assert all(rule.startswith('Read(//') and '*' not in rule for rule in cmd[i+1:i+4])
 assert '--allowedTools' not in a.command({'id':'T-smoke'})
 assert '--permission-mode' in cmd and 'dontAsk' in cmd
 (tmp_path/'AGENTS.md').unlink();(tmp_path/'AGENTS.md').symlink_to(tmp_path/'CLAUDE.md')
 with pytest.raises(ValueError):a.approve_instruction_reads_once('T-smoke',tmp_path)

def test_successful_retry_clears_current_error_preserving_audit(tmp_path):
 from team_factory.core import Store
 s=Store(tmp_path/'db');g=s.create_goal('Goal','claude');s.mode('running')
 t=s.claim({'claude'});s.fail(t['id'],t['invocation'],'Read permission denied',immediate=True)
 s.control(t['id'],'retry');t=s.claim({'claude'});s.finish(t['id'],t['invocation'],response(),{'claude'})
 snapshot=s.snapshot();t=snapshot['tasks'][0]
 assert t['status']=='done' and 'error' not in t
 assert t['errors']==['Read permission denied'] and t['retries']==1
 assert any(e['kind']=='AGENT_FAILED' for e in snapshot['events'])

def test_goal_scoped_execution_gate_preserves_other_agent_work(tmp_path):
 from team_factory.core import Store
 s=Store(tmp_path/'db');g=s.create_goal('Project goal','dots')
 with s.tx() as c:
  g=s.get(c,'goals',g['id']);g['dispatch_blocks']={'claude':'Goal-scoped execution permissions and model baseline unresolved'};s.put(c,'goals',g)
 s.mode('running');t=s.claim({'dots','claude'});assert t['assigned_to']=='dots'
 r=response()|{'action':'handoff','next_agent':'claude','instruction':'Evaluate the next scoped step'}
 s.finish(t['id'],t['invocation'],r,{'dots','claude'})
 assert s.claim({'dots','claude'}) is None
 t=s.snapshot()['tasks'][0]
 assert t['status']=='human_input_required' and t['assigned_to']=='claude'
 assert 'model baseline' in t['error']
