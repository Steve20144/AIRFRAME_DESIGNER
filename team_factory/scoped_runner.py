"""Operator-only one-shot capability. Never constructed from agent/task/API data."""
import asyncio
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import socket
import uuid
import time

TOOL='mcp__goal_preparation__prepare_original_model'

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def regular(path):
    path=Path(path)
    if not path.is_absolute() or not path.is_file() or any(p.is_symlink() for p in (path,*path.parents)):
        raise ValueError('Reviewed path must be a regular file without symlink parents')
    return path

@dataclass(frozen=True)
class Grant:
    task_id:str
    goal_id:str
    bundle_path:str
    sha256:str
    expires_at:float
    deadline:float

    @classmethod
    def reviewed(cls,task_id,goal_id,bundle_path,sha256,ttl_seconds=600):
        if not re.fullmatch(r'T-[A-Za-z0-9_-]+',task_id) or not re.fullmatch(r'G-[A-Za-z0-9_-]+',goal_id):
            raise ValueError('Exact task and goal required')
        if type(ttl_seconds) is not int or not 1<=ttl_seconds<=900:
            raise ValueError('Approval must expire within 900 seconds')
        if not re.fullmatch(r'[0-9a-f]{64}',sha256):raise ValueError('Exact reviewed digest required')
        g=cls(task_id,goal_id,str(bundle_path),sha256,time.time()+ttl_seconds,time.monotonic()+ttl_seconds)
        g.verify();return g

    def verify(self):
        if time.time()>=self.expires_at or time.monotonic()>=self.deadline:
            raise ValueError('One-shot approval expired')
        path=regular(self.bundle_path)
        if digest(path)!=self.sha256:raise ValueError('Reviewed bundle changed')
        b=json.loads(path.read_text())
        prep=path.parents[1]
        if b['goal']!=self.goal_id or b['cwd']!=str(prep) or prep.name!=self.goal_id:
            raise ValueError('Approved goal/worktree mismatch')
        if b['write_roots']!=[str(prep/'preparation_output'),str(prep/'preparation_tmp')]:
            raise ValueError('Only exact preparation output/temp roots are supported')
        for group in ('scripts','controller_files'):
            for name,sha in b[group].items():
                if digest(regular(name))!=sha:raise ValueError('Reviewed executable code changed')
        for name,item in b['executables'].items():
            if str(Path(name).resolve())!=item['resolved'] or digest(regular(item['resolved']))!=item['sha256']:
                raise ValueError('Reviewed runtime binary changed')
        loader=b['sandbox_dependency']
        if digest(regular(loader['path']))!=loader['sha256']:raise ValueError('OS loader policy changed')
        overlay=json.loads(regular(prep/'execution_prep/overlay.json').read_text())
        for row in overlay['files']:
            rel=Path(row['path'])
            if rel.is_absolute() or '..' in rel.parts:raise ValueError('Overlay path escape')
            if digest(regular(prep/rel))!=row['sha256']:raise ValueError('Reviewed source changed')
        for name in ('preparation_tmp','preparation_output'):
            p=prep/name
            if p.exists() or p.is_symlink():
                raise ValueError('Existing preparation effects require explicit audit; no replay')
        return b

class Broker:
    """Host outside agent sandbox; each payload uses the narrower verified OS sandbox."""
    def __init__(self,grant):
        self.grant=grant;self.process=None;self.server=None;self.directory=None
        self.connections=set();self.connected=False;self.watchdog=None
        self._close_lock=asyncio.Lock();self.uncertain=False

    async def start(self,temp):
        b=self.grant.verify();prep=Path(b['cwd'])
        self.directory=Path(temp)/('broker-'+uuid.uuid4().hex[:8]);self.directory.mkdir(mode=0o700)
        # The bridge gets a Unix socket only; no executable grant is persisted in this directory.
        socket_path=self.directory/('s-'+uuid.uuid4().hex[:8])
        self.socket_path=socket_path
        python=b['entrypoint']['command']
        script=prep/'execution_prep/guarded_runner.py'
        argv=[python,'-I','-B',str(script),'serve','--approved-bundle-sha256',self.grant.sha256,
              '--task-id',self.grant.task_id,'--goal-id',self.grant.goal_id,'--expires-at',str(self.grant.expires_at)]
        env={'PATH':'/usr/bin:/bin','HOME':str(self.directory),'TMPDIR':str(self.directory),'LANG':'en_US.UTF-8','PYTHONDONTWRITEBYTECODE':'1'}
        self.process=await asyncio.create_subprocess_exec(*argv,cwd=prep,env=env,
            stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.DEVNULL,start_new_session=True)
        # Pre-bind a unique socket. asyncio's path= branch can unlink a pre-existing
        # socket before bind; passing sock= avoids that deletion entirely.
        listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        try:
            listener.bind(str(socket_path));listener.setblocking(False)
            self.server=await asyncio.start_unix_server(self.connect,sock=listener,limit=16000)
        except BaseException:
            listener.close()
            raise
        os.chmod(socket_path,0o600)
        async def expire():
            await asyncio.sleep(max(0,self.grant.deadline-time.monotonic()))
            await self.close()
        self.watchdog=asyncio.create_task(expire())
        bridge=Path(__file__).with_name('scoped_bridge.py')
        return {'mcpServers':{'goal_preparation':{'command':python,'args':['-I','-B',str(bridge),'--socket',str(socket_path)]}}}

    async def connect(self,reader,writer):
        current=asyncio.current_task();self.connections.add(current)
        try:
            if self.connected or time.time()>=self.grant.expires_at:
                return
            self.connected=True
            async def forward():
                while line:=await reader.readline():
                    if len(line)>10000:raise ValueError('MCP request too large')
                    self.process.stdin.write(line);await self.process.stdin.drain()
                self.process.stdin.close()
            async def backward():
                while line:=await self.process.stdout.readline():
                    writer.write(line);await writer.drain()
            tasks=[asyncio.create_task(forward()),asyncio.create_task(backward())]
            try:
                await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
            finally:
                for t in tasks:t.cancel()
                await asyncio.gather(*tasks,return_exceptions=True)
        finally:
            writer.close();await writer.wait_closed();self.connections.discard(current)
            # Loss of the sole transport revokes access and stops owned work.
            await self.close()

    async def close(self):
        async with self._close_lock:
            await self._close()

    async def _close(self):
        current=asyncio.current_task()
        if self.watchdog and self.watchdog is not current:self.watchdog.cancel()
        if self.server:
            self.server.close();await self.server.wait_closed();self.server=None
        for task in list(self.connections):
            if task is not current:task.cancel()
        if self.process and self.process.returncode is None:
            self.process.send_signal(signal.SIGTERM)
            try:await asyncio.wait_for(self.process.wait(),5)
            except asyncio.TimeoutError:
                # Do not pretend the payload stopped: leave consumed receipts, no automatic replay.
                self.uncertain=True
                self.process.kill();await self.process.wait()
                raise RuntimeError('Broker did not drain; reconcile private outputs before retry')
        # Retain the unique socket inode and invocation directory; descriptor close only.
