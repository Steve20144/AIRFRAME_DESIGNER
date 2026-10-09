"""Session-bound external worker transport. OS filesystem access authenticates local callers.

No provider API, credentials, wake-up daemon, or model substitute is implemented here.
"""
import argparse
import asyncio
import contextlib
import fcntl
import json
import os
from pathlib import Path
import re
import time
import uuid
from .adapters import AgentAdapter, HumanRequired
from .core import result as validate_result


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', value):
        raise ValueError('Invalid worker/agent/invocation identifier')
    return value


class Mailbox:
    def __init__(self, state, agent, ttl=120):
        base=Path(state).resolve()
        self.root = base/'workers'/identifier(agent)
        if (base/'workers').is_symlink() or self.root.is_symlink():
            raise ValueError('Worker directory cannot be a symlink')
        self.ttl = ttl
        self.root.mkdir(parents=True, mode=0o700, exist_ok=True)
        if self.root.is_symlink() or self.root.stat().st_uid != os.getuid():
            raise ValueError('Worker directory must belong to the current OS user')
        os.chmod(self.root, 0o700)

    @contextlib.contextmanager
    def lock(self):
        fd = os.open(self.root/'lock', os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, 'a') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            yield

    def read(self, name):
        path = self.root/name
        if not path.exists(): return None
        fd = os.open(path, os.O_RDONLY|os.O_NOFOLLOW)
        with os.fdopen(fd) as f:
            text = f.read(200000)
            if len(text) >= 200000: raise ValueError('Worker message too large')
            value = json.loads(text)
        if not isinstance(value, dict): raise ValueError('Invalid worker message')
        return value

    def write(self, name, value):
        tmp = self.root/('.'+uuid.uuid4().hex)
        fd = os.open(tmp, os.O_CREAT|os.O_EXCL|os.O_WRONLY, 0o600)
        with os.fdopen(fd, 'w') as f:
            json.dump(value, f); f.flush(); os.fsync(f.fileno())
        tmp.replace(self.root/name)

    def online(self):
        try:
            p = self.read('presence.json')
            return bool(p and p.get('connected') is True and 0 <= time.time()-p['heartbeat_at'] < self.ttl)
        except (OSError, ValueError, KeyError, TypeError): return False

    def authenticate(self, worker):
        identifier(worker)
        p = self.read('presence.json')
        if not p or p.get('worker_id') != worker or not p.get('connected'):
            raise ValueError('Worker does not own this connection')
        return p

    def attach(self, worker):
        identifier(worker)
        with self.lock():
            if self.online(): raise ValueError('A worker is already connected')
            offer = self.read('offer.json')
            if offer and not offer.get('closed'):
                raise ValueError('Unresolved offer or active lease; no worker replacement allowed')
            self.write('presence.json',dict(worker_id=worker,connected=True,heartbeat_at=time.time()))
        return {'worker_id':worker,'heartbeat_seconds':self.ttl,'scope':'active connected-computer session only'}

    def heartbeat(self, worker):
        with self.lock():
            p=self.authenticate(worker)
            if not self.online(): raise ValueError('Heartbeat expired; cannot revive an uncertain lease')
            p['heartbeat_at']=time.time();self.write('presence.json',p)
            offer=self.read('offer.json')
        return {'connected':True,'stop_requested':bool(offer and offer.get('stop_requested')),'invocation':offer.get('invocation') if offer and not offer.get('closed') else None}

    def poll(self, worker):
        with self.lock():
            self.authenticate(worker)
            if not self.online(): raise ValueError('Worker heartbeat expired')
            offer=self.read('offer.json')
            if not offer or offer.get('closed') or offer['worker_id']!=worker:return {'task':None}
            if offer.get('claimed'):return {'task':None,'active_invocation':offer['invocation']}
            offer['claimed']=True;offer['claimed_at']=time.time()
            self.write('offer.json',offer)
            return {'task':offer}

    def complete(self, worker, invocation, lease, result):
        identifier(invocation)
        with self.lock():
            self.authenticate(worker)
            if not self.online():raise ValueError('Heartbeat expired; result cannot release an uncertain workspace')
            offer=self.read('offer.json')
            if not offer or offer.get('closed') or not offer.get('claimed') or offer['worker_id']!=worker or offer['invocation']!=invocation or offer['lease']!=lease:
                raise ValueError('Stale or unowned task lease')
            if self.read('result.json'):raise ValueError('Result already submitted')
            result=validate_result(result, offer.get('allowed_agents', []))
            self.write('result.json',dict(worker_id=worker,invocation=invocation,lease=lease,result=result))
        return {'accepted_for_validation':True}

    def detach(self, worker):
        with self.lock():
            p=self.authenticate(worker)
            offer=self.read('offer.json')
            if offer and offer.get('claimed') and not offer.get('closed'):
                raise ValueError('Complete the claimed task before disconnecting; otherwise let it escalate')
            p['connected']=False;self.write('presence.json',p)
        return {'connected':False}


class ExternalWorkerAdapter(AgentAdapter):
    def __init__(self,*args,state,heartbeat_seconds=120,**kwargs):
        super().__init__(*args,**kwargs)
        self.mailbox=Mailbox(state,self.id,heartbeat_seconds)
        self.pending={}
        self.timeout=900

    def capability(self):
        online=self.mailbox.online()
        return super().capability() | dict(autonomous=online,session_bound=True,
            reason=('Connected external worker; available only while its supervised session heartbeats' if online else
                    'No active external worker. Connect the actual supervised agent session; the daemon cannot wake it.'),
            execution='Existing connected-computer permissions; no model/CLI substitute, no permanent wake-up API')

    def status(self, invocation):
        entry=self.pending.get(invocation)
        if entry:
            try:
                current=self.mailbox.read('offer.json')
                if current and current.get('invocation')==invocation:entry=current
                else:return 'running'
            except (ValueError, OSError):return 'running'
        return 'running' if entry and entry.get('claimed') and not entry.get('closed') else 'idle'

    def stop(self, invocation):
        # Cannot safely kill a remote model turn. Keep its lease; request cooperative stop.
        with self.mailbox.lock():
            o=self.mailbox.read('offer.json')
            if o and o['invocation']==invocation and not o.get('closed'):
                o['stop_requested']=True;self.mailbox.write('offer.json',o);return True
        return False

    async def start_task(self, task, goal, prompt, on_start):
        box=self.mailbox
        with box.lock():
            if not box.online():raise HumanRequired('External worker is disconnected')
            old=box.read('offer.json')
            if old and not old.get('closed'):raise HumanRequired('Unresolved external-worker offer exists')
            presence=box.read('presence.json')
            context=json.loads(prompt)
            offer=dict(allowed_agents=[a['id'] for a in context.get('available_agents',[])],invocation=task['invocation'],task_id=task['id'],goal_id=goal['id'],worker_id=presence['worker_id'],
                       authorization={'source':'user goal submitted to Project Factory','goal_id':goal['id'],'goal_title':goal['title'],'existing_instruction_sources':self.instruction_sources,'worker_must_obey_own_permissions':True},
                       lease=uuid.uuid4().hex,claimed=False,closed=False,stop_requested=False,prompt=json.loads(prompt))
            # Persist invocation/session before publishing work, unlike process spawn there is no child yet.
            on_start(None,'external-'+presence['worker_id'])
            box.write('result.json',{})
            box.write('offer.json',offer)
            self.pending[task['invocation']]=offer
        deadline=time.monotonic()+self.timeout
        while True:
            with box.lock():
                current=box.read('offer.json')
                if not current or current.get('invocation')!=task['invocation']:
                    # Ownership cannot be proven, retain a conservative lease.
                    self.pending[task['invocation']]['claimed']=True
                    raise HumanRequired('Worker offer ownership changed; reconciliation required')
                self.pending[task['invocation']]=current
                response=box.read('result.json')
                if response:
                    if any(response.get(k)!=current[k] for k in ('invocation','lease','worker_id')):
                        raise HumanRequired('Worker returned stale result identity')
                    current['closed']=True;box.write('offer.json',current)
                    self.pending.pop(task['invocation'],None)
                    return response.get('result')
                if not box.online() or time.monotonic()>deadline or (current.get('stop_requested') and not current['claimed']):
                    if not current['claimed']:
                        current['closed']=True;box.write('offer.json',current);self.pending.pop(task['invocation'],None)
                    raise HumanRequired('Worker disconnected, timed out or stopped; no automatic replay')
            await asyncio.sleep(.2)


def main():
    parser=argparse.ArgumentParser(description='Local session-bound Project Factory worker; never starts a model')
    parser.add_argument('--state',type=Path,required=True)
    parser.add_argument('--agent',default='dots')
    parser.add_argument('--worker',required=True,help='Unique identifier for this actual supervised session')
    parser.add_argument('action',choices=['attach','heartbeat','poll','complete','detach'])
    parser.add_argument('--invocation');parser.add_argument('--lease');parser.add_argument('--result',type=Path)
    args=parser.parse_args();os.umask(0o077)
    box=Mailbox(args.state,args.agent)
    if args.action=='complete':
        if not args.result:parser.error('--result required')
        if args.result.stat().st_size>50000:parser.error('Result too large')
        answer=box.complete(args.worker,args.invocation,args.lease,json.loads(args.result.read_text()))
    else:answer=getattr(box,args.action)(args.worker)
    print(json.dumps(answer,ensure_ascii=False))

if __name__=='__main__':main()
