# Connect the actual active Dots session

This is a pull transport for an **already active, supervised connected-computer task**. It never starts Dots, invokes a substitute model, or keeps Dots alive after its session ends. Parent Dots may delegate a bounded connected-computer task using its existing supported delegation interface. That actual task can use this local CLI to receive work and publish results. Future unattended goals still require an active connected session; there is no permanent daemon-to-Dots endpoint.

Use the real delegated task/thread identifier as WORKER below. Do not attach a shell-only timer or mock process and label it Dots. A worker must obey its own existing role, tool permissions and action-time approval requirements, and inspect the goal provenance plus canonical instruction sources in each offer. An offer does not grant additional authority.

From the AIRFRAME_DESIGNER root, with the factory running and before starting a goal:

```sh
.venv/bin/python -m team_factory.worker --state results/factory --worker WORKER attach
.venv/bin/python -m team_factory.worker --state results/factory --worker WORKER heartbeat
.venv/bin/python -m team_factory.worker --state results/factory --worker WORKER poll
```

`poll` atomically claims at most one offer. A second poll returns no new task while that invocation is active. The offer carries goal/task/invocation IDs, the caller's worker/thread ID, a lease fence, the user goal provenance, canonical existing instruction paths, selected Brain context and the assigned worktree. Read those, make only the authorized changes in that worktree and keep heartbeat fresh with explicit supervised calls while working. Default expiry is 120 seconds. Do not run detached heartbeat loops: a background timer cannot prove the actual agent is available.

Create a local JSON result containing exactly:

```json
{"summary":"Explicit public summary", "action":"complete", "next_agent":null, "instruction":"", "tests":[], "limitations":[]}
```

Use `handoff` or `changes` with a registered `next_agent` and an instruction when dictated by the existing roles. Use `blocked` for a genuine blocker/approval. Never claim tests that were not run or place secrets/hidden reasoning in a result.

```sh
.venv/bin/python -m team_factory.worker --state results/factory --worker WORKER complete --invocation INVOCATION --lease LEASE --result /absolute/result.json
```

The runtime validates the exact correlated result and performs ordinary Git evidence collection and automatic continuation. `accepted_for_validation` is a receipt, not a claim that the goal is complete; inspect task state/events to verify completion. Continue `heartbeat`/`poll` during the same bounded connected task if more work is authorized. On idle completion of the supervised task, disconnect:

```sh
.venv/bin/python -m team_factory.worker --state results/factory --worker WORKER detach
```

Heartbeat responses expose cooperative `stop_requested`. The worker should stop safely and return a `blocked` summary. The runtime cannot forcibly kill an external model turn. Lost heartbeat, process restart, or uncertain claimed work retain an orphan lease and require reconciliation; they never automatically reassign or retry potentially active side effects. An expired heartbeat cannot revive an existing uncertain claim. No automatic forced takeover exists.

Private mailbox files live under the state directory with directory mode 0700 and file mode 0600, plus flock/atomic replacement. Existing local OS-user access is the trust boundary, not a newly provisioned credential. The lease UUID is a per-invocation fencing value, not a platform credential. No TCP listener or provider tokens are added. Like other same-user local tools, this cannot cryptographically attest a model identity; only the supervising actual Dots session establishes that provenance. Different users are excluded by filesystem permissions; same-user malicious processes are outside this boundary.

## Current setup verification

The local bridge lifecycle and scheduler integration passed 38 tests. An actual parent-created Dots worker (thread 01a112d1-f4ba-75b1-baae-202a5427b016) then completed read-only goal G-ba2b668fc79c / task T-0befb623ec3a with invocation 49e68a7899cf45e3beaa71c0082281bd. Its exact finding, # AIRFRAME_DESIGNER from the assigned worktree README.md, was independently checked. The worktree remained clean, there was one invocation, and no Claude call. Dispatch was restored to paused; the earlier blocked task was unchanged. Evidence: results/factory/verification/actual-dots-worker-smoke.json. This verifies real session-bound Dots transport, not unattended wake-up or a successful Claude-to-Dots engineering cycle.

Claude remains a separate blocker: supported `claude auth status --json` reported logged in via claude.ai, but the real unchanged-sandbox non-editing smoke returned **401 / expired OAuth** (session `705b9909-f1e4-4750-9888-128bc6e25e06`). Renew authentication through Claude's normal user interface. No credentials were read out or modified during setup.

## Authentication confirmed, instruction-read permission gate

After user reauthentication, the real unchanged-sandbox Claude smoke succeeded (session `9f6e4d76-0725-481a-9b3e-11b056024bb2`). The subsequent two-agent goal `G-5e6a3e30d6b8` stopped before Dots dispatch: Claude Read was denied for canonical `/Users/stefanosfragkoulis/Documents/UtopiaLabs/AIRFRAME_DESIGNER/AGENTS.md`. This is a tool-permission blocker, not an authentication failure. Team paused, no automatic retry, worktree clean. No permission rule was changed. Evidence: `results/factory/verification/two-agent-permission-block.json`.

## Two real agents verified

The same goal `G-5e6a3e30d6b8` / task `T-885e0711d5af` completed after the user approved a one-invocation, exact-file Read allowance for canonical AGENTS.md, CLAUDE.md and brain/HANDOFF.md. Claude returned its own structured `handoff` to dots (event 47), and the scheduler automatically dispatched the actual supervised worker. Dots independently read the same README heading and returned `complete` (event 55), followed by TASK_DONE and GOAL_COMPLETED (56, 57). No source edits or commits. The original permission failure and retry remain in the event history. Evidence: `results/factory/verification/two-real-agent-smoke.json`.

The test-only read grant was held in memory, consumed by one command, and removed by returning to the ordinary startup command. No user/project permission file was edited. It does not authorize future tasks to read external instruction paths automatically. Future permission needs must be approved for their real scope; the default adapter still uses dontAsk and its write sandbox.

For ordinary use:

1. Ask the parent Dots session to connect a real, bounded project worker for your intended goal. It uses the supported connected-computer task interface and its real thread ID, then attach/heartbeat/poll as above.
2. While that worker is connected, enter the goal in the dashboard and select the initial agent according to existing roles. Start it after any actual task-specific permission requirements are resolved.
3. The agents return structured handoffs; the scheduler handles normal transitions without manual relay. Monitor status or pause dispatch from the dashboard.
4. When the supervised session ends, its worker detaches (or heartbeat expires) and becomes unavailable. A later goal needs an active session again; there is no always-on Dots wake-up API.

Claude's current file-tool adapter does not include shell/test-runner/commit automation. This read-only transport proof does not certify arbitrary unattended coding/deployment. Those capabilities remain outside the approved scope. The final factory is paused and the completed worker is detached.

## Expired claimed work: operator salvage

Never revive a stale heartbeat, forge a completion or replay an uncertain workspace. After explicit worker quiescence confirmation and independent process/worktree audit, pause team dispatch. An operator can supply a reviewed JSON audit to `python -m team_factory.reconcile --state results/factory --audit <reviewed.json>`. It verifies exact task/invocation/worker and every saved manifest output hash, validates the ordinary six-field result, records EXTERNAL_RESULT_RECONCILED and applies continuation atomically in SQLite before closing the mailbox offer. The identical audit can finish mailbox cleanup after a crash without replaying the result. Different receipts/results, live tasks, stale identities and changed artifacts are rejected. This is operator-reviewed saved evidence, not a late worker response or renewed lease. Resume only once downstream gates permit. Example actual audit: `results/factory/verification/orphan-dots-reconciliation.json`.
