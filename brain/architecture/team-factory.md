# Autonomous team runtime — 2026-10-06

## Decision and scope

The user requested an executable local orchestration layer replacing manual notification between existing agents. Reuse this project's Python/FastAPI/uvicorn and vanilla frontend stack. SQLite transactions, append-only events and one OS runtime lock provide persistent machine state without a broker or distributed service. `team_factory/` is independent of the aircraft server; port 8097 leaves 8080/PX4 untouched.

Existing roles remain canonical in `brain/HANDOFF.md`, with existing `AGENTS.md`/`CLAUDE.md` instructions. The registry contains source paths, not new implementation/reviewer roles. This note changes no aircraft decisions or physics gates, including the current STEP source corrections and hardware restrictions.

## Architecture

- `core.py`: explicit state machine, goals/tasks, dependencies, atomic claims, append-only events, bounded continuation, cancellation aggregation, escalation, recovery.
- `adapters.py`: generic lifecycle, supported noninteractive Claude transport, unavailable Dots capability, explicitly separate mock demonstration.
- `workspace.py`: retained per-goal worktrees, branch ownership/conflict checks, evidence, bounded canonical Brain selection.
- `runtime.py`: dispatch, context/instruction transport, process/session tracking, graceful shutdown, generated human-readable status.
- `api.py` and `static/`: loopback API/SSE, browser mutation protection and compact original factory dashboard.

A goal receives one initial task. Agents select the next registered agent in a strict structured result; infrastructure does not infer roles, plan product work, or bounce blindly between a fixed pair. The API can add dependent tasks. One writer per goal worktree lets successive turns inspect previous uncommitted changes. Additional concurrency uses separate goals/worktrees. Dirty original checkout content is not copied to new worktrees; this is visible in goal evidence. No automatic merge/push/destruction.

The canonical Brain is selected by path (up to five notes/context budget) with hashes in events. `results/factory/handoff.md` is an automatically generated view; the existing role-bearing handoff is preserved. Long-term project knowledge remains here; execution state lives in ignored runtime SQLite data.

## Current verification and limitations

The deterministic mock loop completes A → B → changes → A → B without manual relay, including restart between turns. Tests also cover transactional concurrency, malformed input, failure/retry/escalation, process cleanup, uncertain recovery, Git conflicts, context escape and HTTP security. The dashboard was exercised in Codex In-app Browser.

Claude Code 2.1.283's installed help and official noninteractive documentation were checked. A real non-editing smoke reached its CLI but returned HTTP 401: existing OAuth authentication expired. The runtime did not renew/change credentials or expand permissions. Dots has no verified supported external trigger in this environment; its adapter is honestly unavailable. The existing project MCP server lets clients access tools; it is not an API to wake the user's Dots identity. These prevent claiming full live two-agent completion.

Claude uses existing roles/instructions and permissions with `dontAsk`, restricted file tools and a macOS write sandbox. Executable customizations/config overrides require security review. The MVP does not enable test-command/commit/deploy/hardware actions; tests claimed by an agent must distinguish unrun checks. A dedicated reviewed execution adapter can extend this later. The synthetic sandbox probe verified intended filesystem boundaries without reading real credentials.

Crash recovery is intentionally conservative: old running tasks retain an orphan lease; an unknown PID cannot be released by the reconciliation CLI. No force retry is offered when a process may still own the workspace. Worktrees are retained, and completion never implies automatic integration.

Run commands, operational details, adapter extension contract and exact limitations: [team_factory/README.md](../../team_factory/README.md).

Primary references checked: [Claude headless execution](https://code.claude.com/docs/en/headless), [settings precedence](https://code.claude.com/docs/en/settings), [settings reference](https://code.claude.com/docs/en/settings-reference), [hooks reference](https://code.claude.com/docs/en/hooks). Installed CLI help remains the compatibility check; no private Dots endpoints were used.

## Session-bound worker setup update, 2026-10-06

User approved connecting the actual active Dots session as a local pull worker. The registry now selects the generic `external_worker` adapter for the existing `dots` identity without changing instruction sources or roles. A supervised connected-computer task attaches using its real task/thread ID, explicitly heartbeats, atomically claims work, and submits a strict invocation/lease-correlated result. Private local files and existing OS-user permissions provide the boundary; no credentials or network endpoint are provisioned. This is not an API that wakes Dots. Availability lasts only while the real session remains connected. Lost claimed-worker heartbeat retains an orphan lease and cannot automatically retry.

The current Claude auth status claims logged in, but the actual non-editing adapter smoke still returned expired OAuth / 401. Normal user reauthentication is still needed. Transport tests must not be described as a successful live two-agent workflow. See [worker operational instructions](../../team_factory/WORKER.md).

Actual Dots transport verification: supervised thread `01a112d1-f4ba-75b1-baae-202a5427b016` completed `G-ba2b668fc79c` / `T-0befb623ec3a`, reading the worktree README heading `# AIRFRAME_DESIGNER`. Result/session/invocation correlation, clean worktree, single dispatch and preservation of earlier blocked work were verified. Team restored to paused. Evidence: `results/factory/verification/actual-dots-worker-smoke.json`. Claude was not invoked in this test; this is a real Dots-only, session-bound success.

Latest setup check: user reauthentication resolved the Claude 401, proven by actual structured-output smoke. Two-real-agent goal `G-5e6a3e30d6b8` then stopped safely on Claude Read permission for canonical `AGENTS.md` outside its isolated worktree. Dots was not dispatched; no permissions were expanded; dispatch restored to paused. Approval for narrowly scoped canonical instruction reads remains a separate decision.

Two-real-agent proof complete: same goal `G-5e6a3e30d6b8` completed after a user-approved one-invocation exact-file Read allowance. Actual Claude structured handoff (event 47) automatically dispatched actual Dots, which independently verified the worktree README and completed (events 55-57). Original permission failure/retry retained, worktree clean, no commits. Test allowance was ephemeral and ordinary startup/default permissions restored. Dots detached; team paused. Evidence: `results/factory/verification/two-real-agent-smoke.json`. Normal use requires an active supervised Dots worker and appropriate task-specific permissions; no unattended wake-up or shell/testing grant was added.
