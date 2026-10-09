import asyncio,hashlib,json,time
from dataclasses import replace
from pathlib import Path
import pytest
from team_factory.scoped_runner import Grant,Broker,TOOL,digest
from team_factory.adapters import ClaudeAdapter,HumanRequired
from team_factory.core import result

@pytest.fixture
def bundle(tmp_path):
 p=tmp_path/'G-test';(p/'execution_prep').mkdir(parents=True)
 code=p/'code.py';code.write_text('# reviewed')
 overlay=p/'execution_prep/overlay.json';overlay.write_text(json.dumps({'files':[{'path':'code.py','sha256':digest(code)}]}))
 b=dict(goal='G-test',cwd=str(p),write_roots=[str(p/'preparation_output'),str(p/'preparation_tmp')],scripts={str(code):digest(code),str(overlay):digest(overlay)},controller_files={},executables={},sandbox_dependency={'path':str(code),'sha256':digest(code)})
 path=p/'execution_prep/permission_bundle.json';path.write_text(json.dumps(b))
 return path,digest(path)

def grant(bundle):return Grant.reviewed('T-test','G-test',*bundle,600)

def test_goal_scope_expiry_and_review_hash(bundle):
 g=grant(bundle)
 with pytest.raises(ValueError):Grant.reviewed('T-test','G-other',*bundle)
 with pytest.raises(ValueError):replace(g,expires_at=0).verify()
 with pytest.raises(ValueError):replace(g,deadline=0).verify()
 with pytest.raises(ValueError):Grant.reviewed('T-test','G-test',*bundle,901)
 bundle[0].write_text('{}')
 with pytest.raises(ValueError,match='bundle changed'):g.verify()

def test_code_change_or_prior_effects_require_rereview(bundle):
 g=grant(bundle);root=bundle[0].parents[1]
 (root/'preparation_tmp').mkdir()
 with pytest.raises(ValueError,match='effects'):g.verify()
 (root/'preparation_tmp').rmdir();(root/'code.py').write_text('# modified')
 with pytest.raises(ValueError,match='code changed'):g.verify()

def test_operator_grant_is_memory_only_and_revocable(bundle):
 a=ClaudeAdapter('claude','Claude',[])
 details=a.approve_preparation_once('T-test','G-test',*bundle)
 assert details['persisted'] is False
 assert not ClaudeAdapter('claude','Claude',[])._scoped_once
 with pytest.raises(ValueError):a.approve_preparation_once('T-test','G-test',*bundle)
 a.revoke_preparation('T-test');assert not a._scoped_once
 assert '--allowedTools' not in a.command({'id':'T-test'})

@pytest.mark.parametrize('failure',['wrong-goal','expired','configuration','spawn'])
def test_dispatch_denial_consumes_grant(bundle,monkeypatch,failure):
 a=ClaudeAdapter('claude','Claude',[]);a.executable='/unused';a.sandbox='/unused'
 a.approve_preparation_once('T-test','G-test',*bundle)
 if failure=='expired':a._scoped_once['T-test']=replace(a._scoped_once['T-test'],expires_at=0)
 task={'id':'T-test','goal_id':'G-test','invocation':'invoke'};goal={'id':'G-test','workspace':str(bundle[0].parents[1])}
 if failure=='wrong-goal':task['goal_id']='G-wrong'
 def check(_):
  if failure=='configuration':raise HumanRequired('blocked configuration')
 monkeypatch.setattr(a,'check_configuration',check)
 if failure=='spawn':
  async def denied(*args):raise PermissionError('synthetic spawn denial')
  monkeypatch.setattr(Broker,'start',denied)
 with pytest.raises((ValueError,RuntimeError,PermissionError)):
  asyncio.run(a.start_task(task,goal,'{}',lambda *args:None))
 assert not a._scoped_once and not a._brokers and not a._instruction_reads_once

def test_no_result_or_config_approval_channel():
 malicious=dict(summary='result',action='complete',next_agent=None,instruction='',tests=[],limitations=[],approve_preparation=True)
 with pytest.raises(ValueError):result(malicious,{'claude'})
 # A task data field cannot affect command construction.
 a=ClaudeAdapter('claude','Claude',[])
 cmd=a.command({'id':'T-test','approve_preparation':True,'scoped_config':{'bad':'value'}})
 assert json.loads(cmd[cmd.index('--mcp-config')+1])=={'mcpServers':{}}
 assert TOOL not in cmd

def test_only_exact_mcp_tool_is_exposed():
 a=ClaudeAdapter('claude','Claude',[])
 config={'mcpServers':{'goal_preparation':{'command':'reviewed-python','args':['fixed-bridge']}}}
 cmd=a.command({'id':'T-test'},config)
 assert json.loads(cmd[cmd.index('--mcp-config')+1])==config
 assert cmd[cmd.index('--allowedTools')+1]==TOOL
 assert cmd[cmd.index('--permission-mode')+1]=='dontAsk'
 assert 'Bash' not in cmd[cmd.index('--tools')+1]
 assert '--strict-mcp-config' in cmd

@pytest.mark.parametrize('close_fails',[False,True])
def test_asgi_stop_active_broker_signals_agent_even_if_close_fails(tmp_path,close_fails):
 import signal
 from team_factory.core import Store
 from team_factory.runtime import Runtime
 from team_factory.api import create_app
 async def run():
  class Process:
   returncode=None
   def __init__(self):self.signals=[]
   def send_signal(self,sig):self.signals.append(sig);self.returncode=-sig
  class ActiveBroker:
   uncertain=False
   def __init__(self):self.closed=asyncio.Event();self.process=Process()
   async def close(self):
    self.closed.set()
    if close_fails:raise RuntimeError('Synthetic close failure')
    self.process.returncode=0
  s=Store(tmp_path/'db');s.create_goal('Stop test','claude');s.mode('running');t=s.claim({'claude'})
  a=ClaudeAdapter('claude','Claude',[]);a._execution_loop=asyncio.get_running_loop()
  agent_process=Process();broker=ActiveBroker()
  a.processes[t['invocation']]=agent_process;a._brokers[t['invocation']]=broker
  runtime=Runtime(s,{'claude':a},None,None,tmp_path)
  app=create_app(runtime)
  async def request(method,path,headers=()):
   messages=[];delivered=False
   async def receive():
    nonlocal delivered
    if not delivered:
     delivered=True;return {'type':'http.request','body':b'','more_body':False}
    await asyncio.Event().wait()
   async def send(message):messages.append(message)
   scope={'type':'http','asgi':{'version':'3.0','spec_version':'2.3'},'http_version':'1.1',
          'method':method,'scheme':'http','path':path,'raw_path':path.encode(),'query_string':b'',
          'root_path':'','headers':[(b'host',b'testserver'),*headers],
          'client':('127.0.0.1',1234),'server':('testserver',80)}
   await asyncio.wait_for(app(scope,receive,send),2)
   status=next(m['status'] for m in messages if m['type']=='http.response.start')
   body=b''.join(m.get('body',b'') for m in messages if m['type']=='http.response.body')
   return status,json.loads(body)
  _,snapshot=await request('GET','/api/state')
  status,response=await request('POST','/api/agents/claude/stop',[(b'x-factory-csrf',snapshot['csrf'].encode())])
  assert status==200 and response=={'signalled':True}
  await asyncio.wait_for(broker.closed.wait(),1)
  await asyncio.sleep(0)
  assert agent_process.signals==[signal.SIGINT]
  assert s.snapshot()['mode']=='paused'
  assert any(e['kind']=='AGENT_STOP_REQUESTED' for e in s.snapshot()['events'])
  if close_fails:
   assert broker.uncertain and a.status(t['invocation'])=='running'
   assert a._stop_failures[t['invocation']]=='RuntimeError'
  else:assert a.status(t['invocation'])=='idle'
 asyncio.run(run())


def test_stop_from_worker_thread_marshals_to_owning_loop():
 import signal
 async def run():
  event=asyncio.Event();loop=asyncio.get_running_loop();signals=[]
  class Process:
   returncode=None
   def send_signal(self,sig):signals.append(sig)
  class ActiveBroker:
   uncertain=False
   async def close(self):
    assert asyncio.get_running_loop() is loop
    event.set()
  a=ClaudeAdapter('claude','Claude',[]);a._execution_loop=loop
  a.processes['i']=Process();a._brokers['i']=ActiveBroker()
  assert await asyncio.to_thread(a.stop,'i')
  await asyncio.wait_for(event.wait(),1)
  assert signals==[signal.SIGINT]
 asyncio.run(run())


def test_stop_without_broker_loop_still_signals_claude():
 import signal
 signals=[]
 class Process:
  returncode=None
  def send_signal(self,sig):signals.append(sig)
 class BrokerWithoutLoop:uncertain=False
 a=ClaudeAdapter('claude','Claude',[]);b=BrokerWithoutLoop()
 a.processes['i']=Process();a._brokers['i']=b
 with pytest.raises(HumanRequired,match='event loop'):a.stop('i')
 assert signals==[signal.SIGINT] and b.uncertain
