import argparse
import json
import os
from pathlib import Path
from .adapters import registry
from .api import create_app
from .core import Store
from .runtime import Runtime
from .workspace import Brain, Workspaces


def main():
    parser = argparse.ArgumentParser(description='Local Project Factory, run from the project root')
    parser.add_argument('--repo', type=Path, default=Path.cwd())
    parser.add_argument('--state', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--port', type=int, default=8097)
    parser.add_argument('--demo', action='store_true', help='Explicit mock agents; no model calls')
    parser.add_argument('--reconcile', metavar='TASK', help='Release an interrupted task only after verifying its process has ended')
    args = parser.parse_args()
    os.umask(0o077)
    root = args.repo.resolve()
    state = (args.state or root / 'results' / ('factory-demo' if args.demo else 'factory')).resolve()
    config = json.loads((args.config or Path(__file__).with_name('config.json')).read_text())
    store = Store(state / 'factory.sqlite3', config.get('limits'))
    if args.reconcile:
        import fcntl
        state.mkdir(parents=True, exist_ok=True)
        with (state / 'runtime.lock').open('a+') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with store.tx() as c:
                t = store.get(c, 'tasks', args.reconcile)
                if not t.get('orphaned'):
                    raise SystemExit('Task has no uncertain invocation')
                if not t.get('pid'):
                    raise SystemExit('No persisted process identity: cannot prove the child ended. Lease remains blocked; do not retry this workspace.')
                if t.get('pid'):
                    try:
                        os.kill(t['pid'], 0)
                    except ProcessLookupError:
                        pass
                    else:
                        raise SystemExit('Recorded PID still exists; refusing to release workspace')
                t['orphaned'] = False
                store.put(c, 'tasks', t)
                store.event(c, 'PROCESS_RECONCILED', t['goal_id'], t['id'], previous_pid=t.get('pid'))
                print('Lease released. Restart the dashboard and explicitly retry the task.')
        return
    adapters = registry(config, args.demo, state=state)
    runtime = Runtime(store, adapters, Workspaces(root, state), Brain(root, store.limits['context_chars']), state)
    import uvicorn
    print(f'Project Factory: http://127.0.0.1:{args.port} ({"MOCK DEMO" if args.demo else "live adapters"})')
    uvicorn.run(create_app(runtime, args.demo), host='127.0.0.1', port=args.port, access_log=False, timeout_graceful_shutdown=3)

if __name__ == '__main__':
    main()
