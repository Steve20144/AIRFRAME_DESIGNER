import asyncio
import json
from team_factory.core import Store
from team_factory.runtime import Runtime
from team_factory.adapters import AgentAdapter, HumanRequired
from team_factory.workspace import Brain


def handoff():
    return dict(summary="Earlier handoff", action="handoff", next_agent="b",
                instruction="Continue", tests=[], limitations=[])


class FailingAdapter(AgentAdapter):
    async def start_task(self, task, goal, prompt, on_start):
        assert json.loads(prompt)["previous_result"] == handoff()
        raise HumanRequired("Current invocation denied")


class ExistingWorkspace:
    async_unused = None
    def prepare(self, goal):
        return goal


def test_adapter_exception_clears_current_result_preserves_history(tmp_path):
    state = tmp_path / "state"
    store = Store(state / "db")
    goal = store.create_goal("Feature", "a")
    with store.tx() as c:
        g = store.get(c, "goals", goal["id"])
        g.update(workspace=str(tmp_path), branch="test", baseline="test")
        store.put(c, "goals", g)
    store.mode("running")
    first = store.claim({"a", "b"})
    store.finish(first["id"], first["invocation"], handoff(), {"a", "b"})
    current = store.claim({"a", "b"})
    assert current["result"] is None
    assert current["previous_result"] == handoff()
    assert current["previous_result_invocation"] == first["invocation"]
    runtime = Runtime(store, {"b": FailingAdapter("b", "B", [])}, ExistingWorkspace(), Brain(tmp_path), state)
    asyncio.run(runtime.run_one(current))
    t = store.snapshot()["tasks"][0]
    assert t["status"] == "human_input_required"
    assert t["result"] is None
    assert t["error"] == "Current invocation denied"
    events = store.snapshot()["events"]
    assert any(e["kind"] == "RESULT_ARCHIVED" and e["data"]["invocation"] == first["invocation"] for e in events)
    assert any(e["kind"] == "AGENT_FINISHED" and e["data"]["result"] == handoff() for e in events)
    restored = Store(state / "db")
    restored.recover()
    assert restored.snapshot()["tasks"][0]["previous_result"] == handoff()
    assert restored.snapshot()["tasks"][0]["result"] is None


def test_claim_clears_stale_error_but_keeps_failure_events(tmp_path):
    s = Store(tmp_path / "db")
    s.create_goal("Feature", "a"); s.mode("running")
    t = s.claim({"a"}); s.fail(t["id"], t["invocation"], "transient")
    t = s.claim({"a"})
    assert "error" not in t and t["result"] is None
    assert t["errors"] == ["transient"]
    assert any(e["kind"] == "AGENT_FAILED" for e in s.snapshot()["events"])
