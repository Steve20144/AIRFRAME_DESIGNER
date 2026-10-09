"""Adapters expose lifecycle, not agent roles. Never persist raw model transcripts."""
import asyncio
import json
import shutil
import signal
import os
import re
import tempfile
from pathlib import Path
import uuid
from abc import ABC, abstractmethod
from .core import RESULT_SCHEMA


class HumanRequired(RuntimeError):
    pass


class AgentAdapter(ABC):
    def __init__(self, id, name, instruction_sources):
        self.id, self.name, self.instruction_sources = id, name, instruction_sources
        self.processes = {}

    def capability(self):
        return dict(id=self.id, name=self.name, autonomous=True,
                    instruction_sources=self.instruction_sources, reason=None)

    @abstractmethod
    async def start_task(self, task, goal, prompt, on_start): ...

    async def resume_task(self, task, goal, prompt, on_start):
        return await self.start_task(task, goal, prompt, on_start)

    def stop(self, invocation):
        p = self.processes.get(invocation)
        if p and p.returncode is None:
            p.send_signal(signal.SIGINT)
            return True
        return False

    def status(self, invocation):
        p = self.processes.get(invocation)
        return 'running' if p and p.returncode is None else 'idle'

    def parse_result(self, output):
        return json.loads(output)


class DotsAdapter(AgentAdapter):
    def capability(self):
        return super().capability() | dict(autonomous=False, reason=
            'No supported local API to wake the existing ChatGPT Dots identity was found. '
            'ChatGPT MCP accesses project tools in the opposite direction. No substitute agent is used.')

    async def start_task(self, task, goal, prompt, on_start):
        raise RuntimeError(self.capability()['reason'])


class ClaudeAdapter(AgentAdapter):
    def __init__(self, *args, executable='claude', timeout=900, **kwargs):
        super().__init__(*args, **kwargs)
        self.executable = shutil.which(executable)
        self.timeout = timeout
        self.sandbox = shutil.which("sandbox-exec")
        self._instruction_reads_once = {}
        self._scoped_once = {}
        self._brokers = {}
        self._execution_loop = None
        self._stop_waiters = set()
        self._stop_failures = {}

    def approve_instruction_reads_once(self, task_id, root):
        """Operator-only in-memory approval, consumed by one command, never loaded from task data."""
        if not isinstance(task_id, str) or not re.fullmatch(r'T-[A-Za-z0-9_-]+', task_id):
            raise ValueError('Exact task identifier required')
        root = Path(root).resolve()
        paths = [root/'AGENTS.md', root/'CLAUDE.md', root/'brain/HANDOFF.md']
        if any(not p.is_file() or p.is_symlink() or p.resolve() != p for p in paths):
            raise ValueError('Instruction approval requires the exact regular canonical files')
        self._instruction_reads_once[task_id] = [str(p) for p in paths]
        return [str(p) for p in paths]

    def approve_preparation_once(self, task_id, goal_id, bundle_path, bundle_sha256, ttl_seconds=600):
        """Trusted operator hook only. No API, config, task or result field calls this."""
        from .scoped_runner import Grant
        if task_id in self._scoped_once:
            raise ValueError('Revoke the previous one-shot approval first')
        grant = Grant.reviewed(task_id, goal_id, bundle_path, bundle_sha256, ttl_seconds)
        self._scoped_once[task_id] = grant
        return dict(task_id=task_id, goal_id=goal_id, expires_at=grant.expires_at, persisted=False)

    def revoke_preparation(self, task_id):
        self._scoped_once.pop(task_id, None)
        self._instruction_reads_once.pop(task_id, None)

    def status(self, invocation):
        broker = self._brokers.get(invocation)
        if broker and (broker.uncertain or (broker.process and broker.process.returncode is None)):
            return 'running'
        return super().status(invocation)

    def stop(self, invocation):
        broker = self._brokers.get(invocation)
        try:
            # Always signal Claude independently of scheduling/closing the broker.
            signalled = super().stop(invocation)
        finally:
            if broker:
                loop = self._execution_loop
                if loop is None or loop.is_closed():
                    broker.uncertain = True
                    raise HumanRequired('Broker event loop unavailable; reconcile its process after stop')
                def schedule_close():
                    async def drain():
                        try:
                            await broker.close()
                        except BaseException as exc:
                            broker.uncertain = True
                            self._stop_failures[invocation] = type(exc).__name__
                    waiter = loop.create_task(drain())
                    self._stop_waiters.add(waiter)
                    waiter.add_done_callback(self._stop_waiters.discard)
                try:
                    loop.call_soon_threadsafe(schedule_close)
                except RuntimeError:
                    broker.uncertain = True
                    raise HumanRequired('Broker stop could not be scheduled; process reconciliation required') from None
        return signalled or bool(broker)

    def capability(self):
        return super().capability() | dict(autonomous=bool(self.executable and self.sandbox),
            reason=('Claude Code executable not found' if not self.executable else
                    'This adapter needs the macOS filesystem sandbox; no unsandboxed fallback' if not self.sandbox else None),
            permissions='dontAsk, existing permissions apply, no bypass flags; restricted built-in file tools')

    def command(self, task, scoped_config=None):
        # Do not replace the system prompt, agent identity, CLAUDE.md discovery or credential mechanism.
        # Tool restriction excludes shell, network and MCP actions from this conservative MVP.
        cmd = [self.executable, '-p', '--output-format', 'json', '--json-schema', json.dumps(RESULT_SCHEMA),
               '--permission-mode', 'dontAsk', '--tools', 'Read,Glob,Grep,Edit,Write', '--max-turns', '20',
               '--strict-mcp-config', '--mcp-config', json.dumps(scoped_config or {'mcpServers': {}})]
        approved = self._instruction_reads_once.pop(task.get('id'), [])
        allowed = []
        if scoped_config:
            from .scoped_runner import TOOL
            allowed.append(TOOL)
        if approved:
            # Claude absolute-path rules begin //; no globs, directories or bare tool grants.
            allowed += ['Read(/' + p + ')' for p in approved]
        if allowed:
            cmd += ['--allowedTools', *allowed]
        if task.get('session') and task.get('session_agent') == self.id:
            cmd += ['--resume', task['session']]
        else:
            cmd += ['--session-id', str(uuid.uuid4())]
        return cmd

    def parse_result(self, output):
        envelope = json.loads(output)
        if not isinstance(envelope, dict) or envelope.get('is_error'):
            raise RuntimeError('Claude returned an error; inspect local Claude session using its identifier')
        if envelope.get('permission_denials'):
            raise HumanRequired('Claude needs tool permission; no permission was automatically expanded')
        structured = envelope.get('structured_output')
        if not structured:
            raise ValueError('Claude did not return the structured continuation result')
        return structured

    @staticmethod
    def sandbox_profile(workspace, temp):
        root = str(Path(workspace).resolve())
        writable = [root, str(Path(temp).resolve())]
        writable += [str((Path.home()/'.claude'/d).resolve()) for d in ('projects', 'debug', 'todos', 'session-env')]
        profile = '(version 1)(allow default)(deny file-write*)(allow file-write* ' + ' '.join('(subpath '+json.dumps(p)+')' for p in writable) + ')(allow file-write-data (literal "/dev/null"))'
        protected = re.escape(root) + r'/([^/]+/)*([.]claude|[.]agents|[.]git|CLAUDE[.]md|AGENTS[.]md|[.]mcp[.]json)(/|$)'
        profile += '(deny file-write-unlink)'
        return profile + '(deny file-write* (regex ' + json.dumps(protected) + ') (literal ' + json.dumps(root+'/brain/HANDOFF.md') + '))'

    def check_configuration(self, workspace):
        # Hooks/helpers can execute outside the built-in tool set. Never silently disable
        # safety hooks or approve executable customizations for autonomous runs.
        configured = os.environ.get('CLAUDE_CONFIG_DIR')
        if configured and Path(configured).expanduser().resolve() != (Path.home()/'.claude').resolve():
            raise HumanRequired('Nondefault CLAUDE_CONFIG_DIR requires explicit adapter configuration review')
        paths = [Path.home()/'.claude/settings.json',
                 Path('/Library/Application Support/ClaudeCode/managed-settings.json')]
        for parent in (Path(workspace), *Path(workspace).parents):
            paths += [parent/'.claude/settings.json', parent/'.claude/settings.local.json']
        paths += list(Path(workspace).glob('**/.claude/settings*.json'))
        paths += list(Path('/Library/Application Support/ClaudeCode/managed-settings.d').glob('*.json'))
        executable_keys = {'hooks', 'statusLine', 'fileSuggestion', 'apiKeyHelper',
                           'enabledPlugins', 'awsAuthRefresh', 'awsCredentialExport',
                           'otelHeadersHelper', 'gcpAuthRefresh', 'forceLoginMethod', 'env'}
        for path in paths:
            if path.exists():
                try:
                    data = json.loads(path.read_text())
                except (ValueError, OSError):
                    raise HumanRequired('Claude configuration cannot be validated; human review required') from None
                if not isinstance(data, dict) or any(data.get(k) for k in executable_keys):
                    raise HumanRequired('Executable Claude customization requires security review before autonomous invocation')

    async def start_task(self, task, goal, prompt, on_start):
        # Consume before validation/spawn; denial or crash cannot leave a latent approval.
        self._execution_loop = asyncio.get_running_loop()
        grant = self._scoped_once.pop(task.get('id'), None)
        broker = None
        p = None
        temp = None
        try:
            if not self.executable or not self.sandbox:
                raise RuntimeError(self.capability()['reason'])
            if grant:
                if task.get('goal_id') != grant.goal_id or goal.get('id') != grant.goal_id:
                    raise HumanRequired('One-shot approval belongs to a different goal')
                bundle = grant.verify()
            self.check_configuration(goal['workspace'])
            # Retain invocation artifacts; never install a TemporaryDirectory finalizer.
            temp = Path(tempfile.mkdtemp(prefix='factory-agent-', dir='/tmp'))
            scoped_config = None
            if grant:
                from .scoped_runner import Broker
                broker = Broker(grant)
                self._brokers[task['invocation']] = broker
                scoped_config = await broker.start(str(temp))
            cmd = self.command(task, scoped_config)
            session = cmd[-1]
            profile = self.sandbox_profile(goal['workspace'], str(temp))
            if grant:
                # Claude/bridge need only connect to the socket and read reviewed code.
                # Broker is managed outside this sandbox; its payloads have their own
                # stricter deny-default sandbox. Do NOT broaden the agent's write roots.
                prep = bundle['cwd']
                profile += '(deny file-write* (subpath ' + json.dumps(prep) + '))'
            env = os.environ.copy()
            env['TMPDIR'] = str(temp)
            p = await asyncio.create_subprocess_exec(self.sandbox, '-p', profile, *cmd, cwd=goal['workspace'], env=env,
                stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL, start_new_session=True)
            self.processes[task['invocation']] = p
            async def communicate():
                p.stdin.write(prompt.encode()); await p.stdin.drain(); p.stdin.close()
                output = bytearray()
                while chunk := await p.stdout.read(65536):
                    output.extend(chunk)
                    if len(output) > 2_000_000:
                        raise RuntimeError('Agent output size limit exceeded')
                await p.wait()
                if p.returncode:
                    from .core import clean
                    try:
                        envelope = json.loads(output)
                        detail = clean(envelope.get('errors') or envelope.get('result') or '') if envelope.get('type') == 'result' else ''
                    except (ValueError, AttributeError):
                        detail = ''
                    if any(v in str(detail).lower() for v in ('authenticate', 'permission', 'credential', 'oauth')):
                        raise HumanRequired(str(detail)[:2000])
                    raise RuntimeError(f'Claude exited with code {p.returncode}; session {session}; {str(detail)[:2000]}')
                return self.parse_result(output)
            on_start(p.pid, session)
            return await asyncio.wait_for(communicate(), self.timeout)
        finally:
            self._instruction_reads_once.pop(task.get('id'), None)
            try:
                if broker:
                    await broker.close()
                    if broker.uncertain:
                        raise RuntimeError('Broker stop was not verified; process reconciliation required')
                    self._brokers.pop(task['invocation'], None)
            finally:
                if p and p.returncode is None:
                    p.send_signal(signal.SIGINT)
                    try:
                        await asyncio.wait_for(p.wait(), 10)
                    except asyncio.TimeoutError:
                        p.terminate()
                        try:
                            await asyncio.wait_for(p.wait(), 10)
                        except asyncio.TimeoutError:
                            raise RuntimeError('Agent did not stop; process reconciliation required')
                if not p or p.returncode is not None:
                    self.processes.pop(task['invocation'], None)
                    # Temp path and any stale socket are retained intentionally, including on denial.
                    # Only owned processes/transports are stopped; no filesystem cleanup.


class MockAdapter(AgentAdapter):
    """Explicit demo only. Persisted transition count makes restarts deterministic."""
    async def start_task(self, task, goal, prompt, on_start):
        on_start(None, 'demo-' + task['invocation'])
        await asyncio.sleep(0.2)
        n = task['transitions']
        actions = [('handoff', 'mock-b', 'Inspect the example package'),
                   ('changes', 'mock-a', 'Correct the example package'),
                   ('handoff', 'mock-b', 'Inspect the corrected package'),
                   ('complete', None, '')]
        action, agent, instruction = actions[min(n, 3)]
        return dict(summary=['Example package prepared', 'Example correction requested',
                              'Example correction prepared', 'Example approved'][min(n, 3)],
                    action=action, next_agent=agent, instruction=instruction,
                    tests=['Deterministic mock protocol'], limitations=['Demo, no real model calls or code edits'])


def registry(config, demo=False, state=None):
    if demo:
        return {id: MockAdapter(id, 'Demo ' + id, []) for id in ('mock-a', 'mock-b')}
    from .worker import ExternalWorkerAdapter
    types = {'claude': ClaudeAdapter, 'dots': DotsAdapter, 'external_worker': ExternalWorkerAdapter}
    agents = {}
    for item in config['agents']:
        if item['adapter'] not in types or item['id'] in agents:
            raise ValueError('Unknown adapter type or duplicate agent ID')
        options = dict(item.get('options', {}))
        if item['adapter'] == 'external_worker':
            if state is None: raise ValueError('External worker requires a runtime state directory')
            options['state'] = state
        agents[item['id']] = types[item['adapter']](item['id'], item['name'], item['instruction_sources'], **options)
    return agents
