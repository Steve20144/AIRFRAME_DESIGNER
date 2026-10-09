"""Transactional SQLite state and bounded, role-neutral continuation protocol."""
import contextlib
import hashlib
import json
import re
import sqlite3
import time
import uuid
from pathlib import Path

STATES = {
    'ready': {'running', 'paused', 'cancelled', 'human_input_required'},
    'running': {'ready', 'done', 'failed', 'human_input_required', 'cancelled'},
    'failed': {'ready', 'cancelled', 'human_input_required'},
    'paused': {'ready', 'cancelled'},
    'human_input_required': {'ready', 'cancelled', 'done'},
    'done': set(), 'cancelled': set(),
}
DEFAULT_LIMITS = dict(max_parallel_agents=1, max_retries=2, max_transitions=12,
                      max_reviews=3, max_identical_errors=2, task_age_seconds=86400,
                      invocation_timeout=900, context_chars=18000)
RESULT_SCHEMA = {
    'type': 'object', 'additionalProperties': False,
    'properties': {
        'summary': {'type': 'string'},
        'action': {'type': 'string', 'enum': ['complete', 'handoff', 'changes', 'blocked']},
        'next_agent': {'type': ['string', 'null']},
        'instruction': {'type': 'string'},
        'tests': {'type': 'array', 'items': {'type': 'string'}},
        'limitations': {'type': 'array', 'items': {'type': 'string'}},
    }, 'required': ['summary', 'action', 'next_agent', 'instruction', 'tests', 'limitations'],
}

def clean(value):
    """Only explicit summaries are stored; raw model/tool transcripts are never logged."""
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()
                if not any(x in k.lower() for x in ('thinking', 'reasoning', 'password', 'token', 'secret', 'api_key'))}
    if isinstance(value, list):
        return [clean(v) for v in value]
    if isinstance(value, str):
        value = re.sub(r'<(thinking|think|analysis|reasoning)(?:\s[^>]*)?>[\s\S]*?(</\1>|$)', '[omitted]', value, flags=re.I)
        value = re.sub(r'(?i)\b(sk-[\w-]+|Bearer\s+\S+)', '[redacted]', value)
        value = re.sub(r'''(?i)([\"']?(?:api[_-]?key|password|secret|token)[\"']?\s*[:=]\s*)([\"'])(?:\\.|(?!\2).)*?\2''', r'\1"[redacted]"', value)
        value = re.sub(r'(?i)((?:api[_-]?key|password|secret|token)\s*[:=]\s*)[^\s,;]+', r'\1[redacted]', value)
        return value[:20000]
    return value


def result(raw, agents):
    if not isinstance(raw, dict) or set(raw) != set(RESULT_SCHEMA['required']):
        raise ValueError('Malformed result: expected the continuation schema')
    if raw['action'] not in ('complete', 'handoff', 'changes', 'blocked'):
        raise ValueError('Unsupported next action')
    for k in ('summary', 'instruction'):
        if not isinstance(raw[k], str) or len(raw[k]) > 12000:
            raise ValueError('Invalid result text')
    for k in ('tests', 'limitations'):
        if not isinstance(raw[k], list) or len(raw[k]) > 50 or any(not isinstance(x, str) or len(x) > 2000 for x in raw[k]):
            raise ValueError('Invalid result list')
    if raw['action'] in ('handoff', 'changes'):
        if not isinstance(raw['next_agent'], str) or raw['next_agent'] not in agents or not raw['instruction'].strip():
            raise ValueError('Continuation requires a registered agent and instruction')
    elif raw['next_agent'] is not None:
        raise ValueError('Terminal action cannot dispatch another agent')
    return clean(raw)


class Store:
    def __init__(self, path, limits=None):
        self.path = str(path)
        self.limits = DEFAULT_LIMITS | (limits or {})
        if set(self.limits) != set(DEFAULT_LIMITS) or any(type(v) is not int or v < 1 for v in self.limits.values()):
            raise ValueError('Limits must be positive integers with known names')
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with self.tx() as c:
            c.executescript('''
            CREATE TABLE IF NOT EXISTS goals(id TEXT PRIMARY KEY, data TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS tasks(id TEXT PRIMARY KEY, data TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS events(seq INTEGER PRIMARY KEY AUTOINCREMENT, at REAL NOT NULL,
              kind TEXT NOT NULL, goal TEXT, task TEXT, data TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS settings(key TEXT PRIMARY KEY,value TEXT NOT NULL);
            INSERT OR IGNORE INTO settings VALUES('mode','paused');
            CREATE TRIGGER IF NOT EXISTS immutable_events_update BEFORE UPDATE ON events
              BEGIN SELECT RAISE(ABORT,'events are append-only'); END;
            CREATE TRIGGER IF NOT EXISTS immutable_events_delete BEFORE DELETE ON events
              BEGIN SELECT RAISE(ABORT,'events are append-only'); END;
            ''')

    @contextlib.contextmanager
    def tx(self):
        c = sqlite3.connect(self.path, timeout=15)
        c.row_factory = sqlite3.Row
        c.execute('PRAGMA journal_mode=WAL')
        c.execute('BEGIN IMMEDIATE')
        try:
            yield c
            c.commit()
        except BaseException:
            c.rollback()
            raise
        finally:
            c.close()

    def event(self, c, kind, goal=None, task=None, **data):
        c.execute('INSERT INTO events(at,kind,goal,task,data) VALUES(?,?,?,?,?)',
                  (time.time(), kind, goal, task, json.dumps(clean(data))))

    def get(self, c, table, id):
        row = c.execute(f'SELECT data FROM {table} WHERE id=?', (id,)).fetchone()
        if not row:
            raise ValueError(f'Unknown {table} id')
        return json.loads(row[0])

    def put(self, c, table, item):
        item['updated_at'] = time.time()
        c.execute(f'INSERT OR REPLACE INTO {table} VALUES(?,?)', (item['id'], json.dumps(item)))

    def all(self, c, table):
        return [json.loads(r[0]) for r in c.execute(f'SELECT data FROM {table}')]

    def transition(self, c, task, status, **data):
        old = task['status']
        if status not in STATES[old]:
            raise ValueError(f'Invalid transition {old} -> {status}')
        task['status'] = status
        task.update(data)
        self.put(c, 'tasks', task)
        self.event(c, 'TASK_' + status.upper(), task['goal_id'], task['id'], previous=old, **data)

    def create_goal(self, title, agent, context=()):
        if not isinstance(title, str) or not title.strip() or len(title) > 10000:
            raise ValueError('A goal needs 1 to 10000 characters')
        with self.tx() as c:
            goal = dict(id='G-' + uuid.uuid4().hex[:12], title=clean(title), status='active',
                        created_at=time.time(), workspace=None, interventions=[], result=None)
            self.put(c, 'goals', goal)
            self.event(c, 'GOAL_CREATED', goal['id'], title=goal['title'])
            self._task(c, goal['id'], title, agent, [], list(context))
            return goal

    def _task(self, c, goal, title, agent, dependencies, context):
        for dep in dependencies:
            if self.get(c, 'tasks', dep)['goal_id'] != goal:
                raise ValueError('Dependencies must belong to the same goal')
        task = dict(id='T-' + uuid.uuid4().hex[:12], goal_id=goal, title=clean(title),
                    assigned_to=agent, status='ready', dependencies=dependencies, brain_context=context,
                    created_at=time.time(), retries=0, transitions=0, reviews=0, errors=[],
                    instruction='', interventions=[], result=None, invocation=None, session=None, session_agent=None,
                    commits=[], files=[], pid=None)
        self.put(c, 'tasks', task)
        self.event(c, 'TASK_CREATED', goal, task['id'], agent=agent, title=task['title'])
        return task

    def create_task(self, goal, title, agent, dependencies=(), context=()):
        with self.tx() as c:
            g = self.get(c, 'goals', goal)
            if g['status'] != 'active':
                raise ValueError('Goal is not active')
            return self._task(c, goal, title, agent, list(dependencies), list(context))

    def mode(self, mode):
        if mode not in ('running', 'paused', 'stopped'):
            raise ValueError('Invalid team mode')
        with self.tx() as c:
            c.execute("UPDATE settings SET value=? WHERE key='mode'", (mode,))
            self.event(c, 'TEAM_' + mode.upper())

    def snapshot(self, after=0):
        with self.tx() as c:
            events = [dict(r) for r in c.execute('SELECT * FROM events WHERE seq>? ORDER BY seq DESC LIMIT 200', (after,))]
            for e in events:
                e['data'] = json.loads(e['data'])
            return dict(mode=c.execute("SELECT value FROM settings WHERE key='mode'").fetchone()[0],
                        goals=self.all(c, 'goals'), tasks=self.all(c, 'tasks'), events=events[::-1])

    def claim(self, available_agents):
        with self.tx() as c:
            if c.execute("SELECT value FROM settings WHERE key='mode'").fetchone()[0] != 'running':
                return None
            tasks = self.all(c, 'tasks')
            running = [t for t in tasks if t['status'] == 'running']
            if len(running) >= self.limits['max_parallel_agents']:
                return None
            # One writer per goal workspace, and one invocation per agent.
            # Uncertain orphan processes retain goal/agent ownership until explicit resolution.
            locked = running + [t for t in tasks if t.get('orphaned')]
            for t in tasks:
                if t['status'] != 'ready' or t['assigned_to'] in {x['assigned_to'] for x in locked} or t['goal_id'] in {x['goal_id'] for x in locked}:
                    continue
                if any(self.get(c, 'tasks', d)['status'] != 'done' for d in t['dependencies']):
                    continue
                if self.get(c, 'goals', t['goal_id'])['status'] != 'active':
                    continue
                goal = self.get(c, 'goals', t['goal_id'])
                reason = goal.get('dispatch_blocks', {}).get(t['assigned_to'])
                if t['assigned_to'] not in available_agents:
                    reason = 'Agent has no supported autonomous runtime configured'
                if time.time() - t['created_at'] > self.limits['task_age_seconds']:
                    reason = 'Task age limit reached'
                if reason:
                    self.transition(c, t, 'human_input_required', error=reason)
                    self.event(c, 'HUMAN_INPUT_REQUIRED', t['goal_id'], t['id'], reason=reason)
                    continue
                # A result belongs to one invocation. Keep continuation context separately
                # so an exception/restart cannot display an earlier success as current.
                if t.get('result') is not None:
                    t['previous_result'] = t['result']
                    t['previous_result_invocation'] = t.get('invocation')
                    self.event(c, 'RESULT_ARCHIVED', t['goal_id'], t['id'],
                               invocation=t.get('invocation'), result=t['result'])
                t['result'] = None
                t.pop('error', None)
                self.transition(c, t, 'running', invocation=uuid.uuid4().hex, started_at=time.time(), pid=None)
                self.event(c, 'AGENT_STARTED', t['goal_id'], t['id'], agent=t['assigned_to'])
                return t

    def finish(self, id, invocation, raw, agents, evidence=None):
        r = result(raw, agents)
        with self.tx() as c:
            t = self.get(c, 'tasks', id)
            if t['status'] != 'running' or t['invocation'] != invocation:
                raise ValueError('Stale or duplicate invocation result')
            self._apply_result(c, t, r, evidence)

    def _apply_result(self, c, t, r, evidence=None):
        id = t['id']
        t.update(evidence or {})
        t['pid'] = None
        t['result'] = r
        t.pop('error', None)  # Current failure is resolved; errors list and append-only events remain.
        t['transitions'] += 1
        t['reviews'] += r['action'] == 'changes'
        self.event(c, 'AGENT_FINISHED', t['goal_id'], id, agent=t['assigned_to'], result=r, **(evidence or {}))
        limited = t['transitions'] >= self.limits['max_transitions'] or t['reviews'] >= self.limits['max_reviews']
        if r['action'] == 'blocked' or (limited and r['action'] != 'complete'):
            error = 'Continuation limit reached' if limited else r['summary']
            if t['status'] == 'human_input_required':
                t['error'] = error
                self.put(c, 'tasks', t)
            else:
                self.transition(c, t, 'human_input_required', error=error)
            self.event(c, 'HUMAN_INPUT_REQUIRED', t['goal_id'], id, reason=t['error'])
        elif r['action'] in ('handoff', 'changes'):
            self.event(c, 'CHANGES_REQUESTED' if r['action'] == 'changes' else 'REVIEW_REQUESTED', t['goal_id'], id,
                       from_agent=t['assigned_to'], to_agent=r['next_agent'])
            self.transition(c, t, 'ready', assigned_to=r['next_agent'], instruction=r['instruction'])
        else:
            self.transition(c, t, 'done')
            self.settle_goal(c, t['goal_id'])

    def reconcile_external_result(self, id, invocation, raw, agents, audit, evidence=None):
        """Operator-only salvage, never a worker lease renewal or synthetic dispatch."""
        r = result(raw, agents)
        required = {'worker_id', 'confirmation', 'process_check', 'artifact_hashes', 'receipt'}
        if not isinstance(audit, dict) or set(audit) != required or any(not audit[k] for k in required):
            raise ValueError('Explicit worker quiescence, process audit and artifact evidence required')
        fingerprint = hashlib.sha256(json.dumps(dict(result=r, audit=audit, evidence=evidence), sort_keys=True).encode()).hexdigest()
        with self.tx() as c:
            if c.execute("SELECT value FROM settings WHERE key='mode'").fetchone()[0] != 'paused':
                raise ValueError('Pause dispatch before operator reconciliation')
            previous = c.execute("SELECT data FROM events WHERE task=? AND kind='EXTERNAL_RESULT_RECONCILED'", (id,)).fetchall()
            for row in previous:
                data = json.loads(row['data'])
                if data.get('invocation') == invocation:
                    if data.get('fingerprint') != fingerprint:
                        raise ValueError('Reconciliation receipt differs from committed result')
                    return {'already_recorded': True, 'fingerprint': fingerprint}
            t = self.get(c, 'tasks', id)
            if t['status'] != 'human_input_required' or not t.get('orphaned') or t.get('invocation') != invocation:
                raise ValueError('Exact orphaned invocation required')
            if t.get('session') != 'external-' + audit['worker_id'] or t.get('pid') is not None:
                raise ValueError('External worker identity mismatch or process remains')
            t['orphaned'] = False
            self.event(c, 'EXTERNAL_RESULT_RECONCILED', t['goal_id'], id,
                       invocation=invocation, fingerprint=fingerprint, audit=audit)
            self._apply_result(c, t, r, evidence)
            return {'already_recorded': False, 'fingerprint': fingerprint}

    def fail(self, id, invocation, error, immediate=False):
        with self.tx() as c:
            t = self.get(c, 'tasks', id)
            if t['status'] != 'running' or t['invocation'] != invocation:
                return
            error = clean(error)
            t['errors'].append(error)
            t['retries'] += 1
            self.event(c, 'AGENT_FAILED', t['goal_id'], id, error=error)
            self.transition(c, t, 'failed', error=error, pid=None)
            exhausted = immediate or t['retries'] > self.limits['max_retries'] or t['errors'].count(error) >= self.limits['max_identical_errors']
            self.transition(c, t, 'human_input_required' if exhausted else 'ready')
            if exhausted:
                self.event(c, 'HUMAN_INPUT_REQUIRED', t['goal_id'], id, reason=error if immediate else 'Failure limit reached')

    def recover(self):
        with self.tx() as c:
            for t in self.all(c, 'tasks'):
                if t['status'] == 'running':
                    self.transition(c, t, 'human_input_required', orphaned=True,
                                    error='Interrupted dispatch. Confirm previous process has ended before retrying; no automatic replay.')
                    self.event(c, 'HUMAN_INPUT_REQUIRED', t['goal_id'], t['id'], reason='Recovery requires process reconciliation', pid=t.get('pid'))
            self.event(c, 'RUNTIME_RECOVERED')

    def control(self, id, action, instruction='', agent=None):
        with self.tx() as c:
            t = self.get(c, 'tasks', id)
            if action == 'instruction':
                t['interventions'].append(clean(instruction))
                self.put(c, 'tasks', t)
                self.event(c, 'HUMAN_INTERVENTION', t['goal_id'], id, instruction=instruction, delivery='next invocation')
                return
            if t['status'] == 'running':
                raise ValueError('Active invocation owns this task. Stop the agent first or pause team dispatch.')
            if t.get('orphaned'):
                raise ValueError('Reconcile the interrupted process with the CLI before releasing its workspace')
            status = {'pause': 'paused', 'cancel': 'cancelled', 'retry': 'ready', 'reassign': 'ready'}.get(action)
            if not status:
                raise ValueError('Unknown control')
            if agent:
                t['assigned_to'] = agent
            if instruction:
                t['instruction'] = clean(instruction)
            if t['status'] == status and action == 'reassign':
                self.put(c, 'tasks', t)
                self.event(c, 'TASK_ASSIGNED', t['goal_id'], id, agent=agent)
            else:
                self.transition(c, t, status)
            if action == 'cancel':
                siblings = [x for x in self.all(c, 'tasks') if x['goal_id'] == t['goal_id']]
                cancelled = {x['id'] for x in siblings if x['status'] == 'cancelled'}
                changed = True
                while changed:
                    changed = False
                    for dependent in siblings:
                        if dependent['status'] not in ('done', 'cancelled', 'running') and cancelled.intersection(dependent['dependencies']):
                            self.transition(c, dependent, 'cancelled', error='Prerequisite cancelled')
                            cancelled.add(dependent['id']); changed = True
                self.settle_goal(c, t['goal_id'])

    def settle_goal(self, c, goal_id):
        siblings = [x for x in self.all(c, 'tasks') if x['goal_id'] == goal_id]
        if not siblings or not all(x['status'] in ('done', 'cancelled') for x in siblings):
            return
        g = self.get(c, 'goals', goal_id)
        if g['status'] != 'active':
            return
        cancelled = any(x['status'] == 'cancelled' for x in siblings)
        summaries = [(x.get('result') or {}).get('summary', '') for x in siblings if x['status'] == 'done']
        g.update(status='cancelled' if cancelled else 'done', result=dict(
            outcome='Partially or fully cancelled by user; work retained' if cancelled else '\n'.join(summaries),
            tasks=[x['id'] for x in siblings],
            tasks_completed=[x['id'] for x in siblings if x['status'] == 'done'],
            tests=[v for x in siblings for v in (x.get('result') or {}).get('tests', [])],
            limitations=[v for x in siblings for v in (x.get('result') or {}).get('limitations', [])],
            commits=sorted({v for x in siblings for v in x['commits']}),
            files=sorted({v for x in siblings for v in x['files']}),
            integration='Work retained in isolated branch; no automatic merge or push'))
        self.put(c, 'goals', g)
        self.event(c, 'GOAL_CANCELLED' if cancelled else 'GOAL_COMPLETED', g['id'], summary=g['result'])
