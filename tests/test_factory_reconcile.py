import hashlib,json
import pytest
from team_factory.core import Store
from team_factory.worker import Mailbox
from team_factory.reconcile import reconcile,digest

@pytest.fixture
def recovery(tmp_path):
 s=Store(tmp_path/"db");g=s.create_goal("Full goal","dots");s.mode("running");t=s.claim({"dots","claude"})
 w=tmp_path/"workspace";w.mkdir();(w/"report.md").write_text("Saved evidence")
 m={"goal_id":g["id"],"worker_id":"session","outputs":[{"path":"report.md","sha256":digest(w/"report.md")}]}
 (w/"manifest.json").write_text(json.dumps(m))
 with s.tx() as c:
  g["workspace"]=str(w);s.put(c,"goals",g)
  t["session"]="external-session";s.put(c,"tasks",t)
 s.recover();s.mode("paused")
 b=Mailbox(tmp_path,"dots");b.attach("session")
 b.write("offer.json",dict(task_id=t["id"],goal_id=g["id"],invocation=t["invocation"],worker_id="session",claimed=True,closed=False,allowed_agents=["dots","claude"],lease="old"))
 b.write("presence.json",dict(worker_id="session",connected=True,heartbeat_at=0))
 args=dict(task_id=t["id"],invocation=t["invocation"],worker="session",manifest_rel="manifest.json",manifest_hash=digest(w/"manifest.json"),raw=dict(summary="Operator reviewed saved report",action="handoff",next_agent="claude",instruction="Review saved report within scope",tests=[],limitations=["Not full goal completion"]),confirmation="Worker confirms quiescent",process_check="No matching active process",evidence={"files":["report.md"],"commits":[]})
 return s,b,w,args

def test_salvage_idempotent_no_replay(recovery):
 s,b,w,a=recovery
 assert not reconcile(s,b,**a)["already_recorded"]
 t=s.snapshot()["tasks"][0]
 assert t["status"]=="ready" and t["assigned_to"]=="claude" and not t["orphaned"]
 assert s.snapshot()["goals"][0]["status"]=="active"
 assert b.read("offer.json")["closed"] and not b.online()
 assert reconcile(s,b,**a)["already_recorded"]
 with pytest.raises(ValueError):b.complete("session",a["invocation"],"old",a["raw"])
 with pytest.raises(ValueError):s.finish(a["task_id"],a["invocation"],a["raw"],{"claude"})
 assert sum(e["kind"]=="EXTERNAL_RESULT_RECONCILED" for e in s.snapshot()["events"])==1

@pytest.mark.parametrize("change",["running","hash","invocation","confirmation","artifact"])
def test_recovery_fails_closed(recovery,change):
 s,b,w,a=recovery
 if change=="running":s.mode("running")
 elif change=="hash":a["manifest_hash"]="wrong"
 elif change=="invocation":a["invocation"]="wrong"
 elif change=="confirmation":a["confirmation"]=""
 else:(w/"report.md").write_text("changed")
 with pytest.raises(ValueError):reconcile(s,b,**a)
 assert s.snapshot()["tasks"][0]["orphaned"]
 assert not b.read("offer.json")["closed"]

def test_crash_between_database_and_mailbox(recovery,monkeypatch):
 s,b,w,a=recovery;original=b.write
 def crash(name,value):
  if name=="offer.json":raise OSError("simulated crash after SQLite commit")
  return original(name,value)
 monkeypatch.setattr(b,"write",crash)
 with pytest.raises(OSError):reconcile(s,b,**a)
 assert not b.read("offer.json")["closed"]
 monkeypatch.setattr(b,"write",original)
 assert reconcile(s,b,**a)["already_recorded"]
 assert b.read("offer.json")["closed"]

def test_blocked_salvage_keeps_task_blocked(recovery):
 s,b,w,a=recovery;a["raw"].update(action="blocked",next_agent=None)
 reconcile(s,b,**a)
 t=s.snapshot()["tasks"][0]
 assert t["status"]=="human_input_required" and not t["orphaned"]
