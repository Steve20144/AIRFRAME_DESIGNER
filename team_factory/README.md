# Project Factory

**Live transport verified:** the real Claude adapter handed a read-only task to an actual supervised Dots worker, which independently verified it and completed the goal. Authentication is working. The three canonical instruction reads were approved for one smoke invocation only; no persistent permission rule was added. Dots availability remains session-bound. See [WORKER.md](WORKER.md) for the actual connection lifecycle and current limits. Earlier audit results below are historical.

Local, persistent coordination infrastructure for the existing project agents. No agent roles are defined here. The registry points to the existing `AGENTS.md`, `CLAUDE.md`, and canonical `brain/HANDOFF.md` role definitions. Project knowledge remains in `brain/`.

**Current live blockers:** no supported external invocation interface was found for the existing ChatGPT Dots identity. Its adapter is explicitly unavailable, not replaced by a mock or another CLI. The installed Claude Code 2.1.283 passed invocation startup but the bounded non-editing smoke returned HTTP 401 / expired OAuth authentication. The user must renew their existing Claude authentication through its normal interface. No credentials or permissions were changed. A successful two-real-agent workflow is therefore **not yet verified**.

## Start

From the AIRFRAME_DESIGNER repository root:

```sh
.venv/bin/python -m team_factory
```

Open **http://127.0.0.1:8097**. This separate FastAPI service does not touch the existing aircraft UI on 8080 or PX4. It starts paused on first use. Enter a goal, choose the initial agent using the existing role definitions, optionally select up to five canonical Brain paths, and press START. The agent's validated structured result names the next registered agent; the scheduler dispatches it automatically. There is no hard-coded implementation/reviewer pairing.

For an explicitly labelled deterministic demonstration, with a separate database and no model calls:

```sh
.venv/bin/python -m team_factory --demo --port 8098
```

The demo performs mock A → mock B → changes requested → mock A → mock B → complete. It makes no engineering changes. Actual and demo state are separate (`results/factory/` and `results/factory-demo/`). `--state /absolute/path` and `--config /absolute/config.json` override these. Do not mix demo and live registries in the same state directory.

## Controls

- **Pause / Stop run:** stop new dispatch; active invocations drain safely. Stop persists across restarts. Resume permits new dispatch. Stop run does not cancel existing tasks.
- **Stop agent:** pause dispatch and signal only that adapter-owned invocation with SIGINT. Timeout cleanup escalates to SIGTERM after 10 seconds, waits another 10, then retains the lease if it cannot prove termination. Unrelated processes are never signalled.
- **Intervention:** recorded immediately and delivered at the next invocation boundary; it does not silently rewrite an active prompt.
- **Task pause / retry / cancel / reassign:** allowed only by the state machine. Active or orphaned work cannot be reassigned. Cancellation propagates through dependents; a goal containing cancelled tasks ends as cancelled, never falsely complete.
- **⌘K / Ctrl-K:** goal, team controls, task, agent, Brain, blockers, activity.
- **Ctrl-C server:** stops dispatch, then drains active work (up to the configured invocation timeout). A forced process kill is treated as uncertain recovery on restart.

## Persistence and lifecycle

`factory.sqlite3` contains goals, tasks, settings, and an append-only events table. Transactions use `BEGIN IMMEDIATE`, invocation IDs reject duplicate/stale results, and an OS file lock excludes a second runtime over the same state directory. Event update/delete triggers reject mutation. All task status changes emit events. SSE has sequence IDs, ordered pagination, and reconnect cursors. Events are retained in SQLite; the dashboard displays the latest 200 and inspectors up to 70 recent matching events.

State is explicit: ready → running → ready/done/failed/human_input_required, with paused and cancelled side states. Dependencies must be completed before dispatch. The scheduler has one invocation per agent and one writer per goal worktree. `max_parallel_agents` is configurable and defaults to **1**. Retries, identical errors, review changes, transitions, task age and invocation duration are bounded in `config.json`. Authentication, configuration security review, permissions and workspace conflicts escalate immediately. Ordinary adapter failures have bounded retries.

On restart, previously running tasks become `human_input_required` with an orphan lease. They are **never automatically replayed**. Stop the server and use:

```sh
.venv/bin/python -m team_factory --reconcile T-xxxxxxxxxxxx
```

This refuses release if the recorded PID exists. It also refuses when no PID was durably recorded: a crash can occur between spawning and persisting the PID, and absence of a PID is not proof of absence of a process. That case requires operator process reconciliation; this MVP intentionally offers no unsafe force-release button. Preserve the state and worktree; do not start another agent in that workspace. A PID reused by another process also keeps the task blocked conservatively. Successfully reconciled tasks still require an explicit Retry in the dashboard.

Generated `results/factory/handoff-snapshots/handoff-<unique-id>.md` files are retained observability views. Existing `results/factory/handoff.md` and `handoff.tmp` are preserved. The existing canonical `brain/HANDOFF.md` is not overwritten or used as the scheduler.

## Agent contract and extension

`AgentAdapter` exposes `start_task`, `resume_task`, `stop`, `status`, `parse_result`, and `capability`. Configure IDs, display names, instruction-source paths and adapter options in `config.json`. Implement another adapter and register its type in `adapters.registry`; the scheduler needs no agent-specific logic. The adapter must enforce its own execution boundaries and cancellation semantics.

Validated result fields are `summary`, `action` (`complete`, `handoff`, `changes`, `blocked`), `next_agent`, `instruction`, `tests`, and `limitations`. Unknown fields, malformed types, unregistered destinations and terminal results that name a next agent are rejected. Handoffs stay within the same goal/task and worktree; they cannot issue shell commands or mutate arbitrary scheduler state. Agents decide continuation using their existing instructions. The infrastructure makes no product decisions. Agent-provided summaries/tests are claims, not independently certified test results.

Tasks can be added with `POST /api/tasks` (same-goal dependencies). The MVP creates one task for a submitted goal; it does not invent an intelligent task planner. Select relevant Brain paths in the UI. Agents may read additional canonical documents if their permissions allow. A later supported Dots runtime should implement the existing adapter lifecycle and structured result contract; there is no invented callback endpoint, browser scraping, session token extraction, or undocumented private API.

## Claude execution boundary

Verified installed CLI options: noninteractive `-p`, JSON output/schema, explicit `--session-id` and `--resume`, `--permission-mode dontAsk`, built-in file tools, max turns, strict empty MCP configuration. Per-agent session IDs survive handoffs. Project instructions are preserved; no replacement system prompt or role definition is supplied. CLI capability means the executable/sandbox are present, not that authentication is healthy.

This conservative MVP exposes Read/Glob/Grep/Edit/Write only. It does **not** enable shell execution, agent-initiated test runners, Git commit/push, network tools, deployment, hardware, or MCP actions. Consequently an ordinary coding goal may require a future explicitly reviewed test/commit adapter. It must report unrun tests honestly. Do not broaden permissions simply to make a demo pass.

Tool selection alone is not a sandbox. A macOS `sandbox-exec` profile additionally denies writes outside the assigned worktree, private temporary directory, and existing Claude session/debug/todo/session-env directories. Workspace `.claude`, `.agents`, `.git`, `CLAUDE.md`, `AGENTS.md`, `.mcp.json`, and the role handoff are protected against writes. Canonical configuration/credential writes are not allowed. A synthetic OS probe verified workspace/session writes succeed and outside/config/instruction writes fail.

Hooks, helper commands, environment-injecting settings and enabled plugins can execute independently of the model's tool list. The adapter checks known user/ancestor/nested-project/managed settings before invocation and refuses executable customizations for security review instead of disabling safety hooks. Nondefault `CLAUDE_CONFIG_DIR` also fails closed. Strict empty MCP configuration prevents starting project-configured MCP servers. This is not a general hostile-code container: Claude still uses the existing OS credential mechanism and network to reach its provider. New CLI versions/configuration surfaces need review before expanding this boundary. Secrets should never be supplied as goal or intervention text.

The dashboard receives only schema-approved results, observed Git evidence and sanitized events. It never loads native Claude transcripts or hidden reasoning streams. Redaction covers structured sensitive keys, common labelled/quoted credentials and reasoning XML; it cannot identify every arbitrary unlabeled secret in prose. Native CLI session files remain managed by Claude itself and are not dashboard logs.

## Git and Brain

Each goal owns `factory/G-…` and a retained Git worktree under the state directory. Turns within that goal are sequential so a review sees uncommitted work. Separate goals can run in separate worktrees. New worktrees start at **committed HEAD**; dirty main-checkout changes are preserved and omitted, with that limitation shown in goal evidence. No automatic checkout, merge, push, deletion, cleanup or integration occurs. Goal complete means the agent marked its isolated work complete; integration status remains explicit in the final result.

Brain excerpts resolve under canonical `brain/`, reject traversal/symlink escapes, cap selection at five documents, enforce a size/context budget and record hashes/paths. Changed selected documents emit `BRAIN_UPDATED`. The Brain button lists recent canonical documents; runtime events/SQLite are not a duplicate project knowledge base. An agent's proposed Brain edits in its worktree still require normal integration.

## Security and observability

Bind is hard-coded to loopback. Browser mutations require a per-process CSRF header; cross-origin/Host checks, request limits, CSP and no-store headers apply. This is a single-user local service, not a multi-user network service. Do not reverse-proxy or expose it publicly. State directories created by the CLI use a private umask. No launch-at-login daemon is installed.

Events include task transitions, adapter invocation/session IDs, selected context, branches, commits, errors, human interventions and completion. UI inspectors show normal explicit summaries/evidence, not hidden chain-of-thought. Use the native CLI to inspect its own session when needed. Goals and historical worktrees remain available after restart.

## Verification

```sh
.venv/bin/python -m pytest tests/test_team_factory.py -q
```

Tests exercise transitions/events, dependencies, duplicate dispatch/result rejection, concurrent claims/agent leases, pause/stop/drain/resume, retry/loop/age bounds, immediate escalation, restart and orphan recovery, null-PID reconciliation refusal, callback failure cleanup, timeout cleanup, malformed result/body, secret/reasoning sanitization, context traversal/budget, dirty-worktree isolation, real Git conflicts, configuration gates, CSRF/Origin protection and deterministic A→B→changes→A→B completion across a restart.

Browser verification uses Codex In-app Browser. Actual Claude authentication and Dots autonomous invocation remain the blockers above; mock success must not be presented as two real agents working.


### Invocation-result history update

New dispatches clear the current result and current error. The preceding explicit result is retained as previous_result for continuation prompts and in append-only RESULT_ARCHIVED events with its old invocation ID. This prevents a resumed adapter failure from displaying a stale outcome as the current invocation. Regression: tests/test_factory_result_history.py. Historical authentication/blocker statements above describe earlier audit checkpoints; the live transport verification at the top supersedes them.
