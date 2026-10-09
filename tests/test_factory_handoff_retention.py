import asyncio
from pathlib import Path
from types import SimpleNamespace
import pytest
from team_factory.core import Store
from team_factory.runtime import Runtime
from team_factory.adapters import AgentAdapter
from team_factory.workspace import Brain

class FinishedAgent(AgentAdapter):
 async def start_task(self,task,goal,prompt,on_start):
  on_start(None,'synthetic-retention-test')
  return dict(summary='Synthetic bounded task finished',action='complete',next_agent=None,instruction='',tests=[],limitations=[])

class WorkspaceStub:
 def __init__(self,path):self.path=path
 def prepare(self,goal):return dict(goal,workspace=str(self.path),branch='test',base_commit='test',baseline={})
 def evidence(self,goal):return dict(commits=[],files=[])

def setup(tmp_path):
 state=tmp_path/'state';store=Store(state/'db');(state/'handoff.md').write_text('original generated view');(state/'handoff.tmp').write_text('existing temporary evidence')
 runtime=Runtime(store,{'a':FinishedAgent('a','A',[])},WorkspaceStub(tmp_path),Brain(tmp_path),state)
 return store,runtime,state

def test_run_one_retains_old_views_and_writes_unique_snapshots(tmp_path):
 store,runtime,state=setup(tmp_path)
 store.create_goal('Bounded task','a');store.mode('running');task=store.claim({'a'})
 asyncio.run(runtime.run_one(task))
 assert store.snapshot()['tasks'][0]['status']=='done'
 first=list((state/'handoff-snapshots').glob('*.md'));assert len(first)==1
 content=first[0].read_bytes();second=runtime.write_handoff_snapshot()
 assert second!=first[0] and first[0].read_bytes()==content
 assert (state/'handoff.md').read_text()=='original generated view'
 assert (state/'handoff.tmp').read_text()=='existing temporary evidence'
 assert len(list((state/'handoff-snapshots').glob('*.md')))==2
 assert sum(e['kind']=='HANDOFF_SNAPSHOT_CREATED' for e in store.snapshot()['events'])==2

def test_snapshot_collision_fails_without_overwriting(tmp_path,monkeypatch):
 _,runtime,state=setup(tmp_path)
 directory=state/'handoff-snapshots';directory.mkdir();prior=directory/'handoff-fixed.md';prior.write_text('preserve')
 monkeypatch.setattr('team_factory.runtime.uuid.uuid4',lambda:SimpleNamespace(hex='fixed'))
 with pytest.raises(FileExistsError):runtime.write_handoff_snapshot()
 assert prior.read_text()=='preserve'

def test_snapshot_symlink_directory_is_refused(tmp_path):
 _,runtime,state=setup(tmp_path);outside=tmp_path/'outside';outside.mkdir()
 (state/'handoff-snapshots').symlink_to(outside,target_is_directory=True)
 with pytest.raises(ValueError,match='symlink'):runtime.write_handoff_snapshot()
 assert list(outside.iterdir())==[]
