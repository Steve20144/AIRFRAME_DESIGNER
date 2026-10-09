"""Git isolation and bounded reads of canonical Brain documents."""
import hashlib
import subprocess
from pathlib import Path


def git(root, *args):
    p = subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True, timeout=45)
    if p.returncode:
        raise RuntimeError('Git operation failed: ' + p.stderr[:1000])
    return p.stdout.strip()


class WorkspaceError(RuntimeError):
    pass


class Workspaces:
    def __init__(self, root, state):
        self.root = Path(root).resolve()
        self.base = Path(state).resolve() / 'worktrees'
        self.base.mkdir(parents=True, exist_ok=True)

    def prepare(self, goal):
        path = self.base / goal['id']
        branch = 'factory/' + goal['id']
        if goal.get('workspace'):
            if Path(goal['workspace']).resolve() != path or not path.is_dir():
                raise WorkspaceError('Workspace missing or ownership mismatch; retained for recovery')
            if git(path, 'branch', '--show-current') != branch:
                raise WorkspaceError('Workspace branch changed; human reconciliation required')
            return goal
        if path.exists():
            raise WorkspaceError('Unregistered workspace exists; refusing to overwrite it')
        head = git(self.root, 'rev-parse', 'HEAD')
        dirty = bool(git(self.root, 'status', '--porcelain', '--untracked-files=no'))
        git(self.root, 'worktree', 'add', '-b', branch, str(path), head)
        return goal | dict(workspace=str(path), branch=branch, base_commit=head,
                           source_dirty=dirty, baseline='Committed HEAD only; original dirty changes are not copied')

    def evidence(self, goal):
        path = Path(goal['workspace'])
        if git(path, 'branch', '--show-current') != goal['branch']:
            raise WorkspaceError('Agent changed the workspace branch')
        if git(path, 'ls-files', '-u'):
            raise WorkspaceError('Git workspace has unresolved conflicts')
        commits = git(path, 'rev-list', goal['base_commit'] + '..HEAD').splitlines()
        files = set(git(path, 'diff', '--name-only', goal['base_commit']).splitlines())
        files.update(git(path, 'ls-files', '--others', '--exclude-standard').splitlines())
        return dict(commits=commits, files=sorted(files))


class Brain:
    def __init__(self, root, budget=18000):
        self.root = Path(root).resolve()
        self.budget = budget

    def resolve(self, relative):
        if not isinstance(relative, str):
            raise ValueError('Invalid context path')
        p = (self.root / relative).resolve()
        if not p.is_relative_to(self.root / 'brain') or p.suffix != '.md' or not p.is_file():
            raise ValueError('Context must be an existing Markdown file inside canonical brain/')
        return p

    def select(self, paths):
        if not isinstance(paths, (list, tuple)) or len(paths) > 5 or any(not isinstance(p, str) for p in paths):
            raise ValueError('Select at most five Brain documents')
        out, budget = [], self.budget
        for name in dict.fromkeys(paths):
            p = self.resolve(name)
            if p.stat().st_size > 1000000:
                raise ValueError('Brain document exceeds read limit')
            content = p.read_text()
            excerpt = content[:budget]
            out.append(dict(path=name, sha256=hashlib.sha256(content.encode()).hexdigest(),
                            text=excerpt, truncated=len(excerpt) < len(content)))
            budget -= len(excerpt)
        return out

    def listing(self):
        return sorted([dict(path=str(p.relative_to(self.root)), updated_at=p.stat().st_mtime)
                       for p in (self.root / 'brain').rglob('*.md')
                       if p.resolve().is_relative_to(self.root / 'brain')], key=lambda x: x['updated_at'], reverse=True)[:300]
