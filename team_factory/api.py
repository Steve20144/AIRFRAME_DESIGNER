import asyncio
import json
import secrets
from contextlib import asynccontextmanager
from pathlib import Path
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import FileResponse, StreamingResponse, JSONResponse
from .core import clean


def create_app(runtime, demo=False):
    store = runtime.store
    csrf = secrets.token_urlsafe(32)
    @asynccontextmanager
    async def lifespan(app):
        runtime.acquire()
        scheduler = asyncio.create_task(runtime.loop())
        try:
            yield
        finally:
            runtime.closing = True
            await scheduler
            await runtime.close()
    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)

    @app.middleware('http')
    async def local_guard(request, call_next):
        host = request.headers.get('host', '')
        if host.split(':')[0] not in ('127.0.0.1', 'localhost', 'testserver'):
            return JSONResponse({'detail': 'Loopback host required'}, status_code=403)
        origin = request.headers.get('origin')
        if origin and origin not in ('http://' + host, 'https://' + host):
            return JSONResponse({'detail': 'Cross-origin request denied'}, status_code=403)
        if request.headers.get('sec-fetch-site') == 'cross-site':
            return JSONResponse({'detail': 'Cross-site request denied'}, status_code=403)
        if request.method not in ('GET', 'HEAD'):
            if request.headers.get('x-factory-csrf') != csrf:
                return JSONResponse({'detail': 'Missing request token'}, status_code=403)
            body = await request.body()
            if len(body) > 50000:
                return JSONResponse({'detail': 'Request too large'}, status_code=413)
        response = await call_next(request)
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Content-Security-Policy'] = "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'"
        return response

    @app.exception_handler(ValueError)
    async def bad_request(request, exc):
        return JSONResponse({'detail': str(exc)}, status_code=400)

    @app.get('/')
    def index():
        return FileResponse(Path(__file__).parent / 'static/index.html')

    @app.get('/assets/{name}')
    def asset(name: str):
        if name not in ('app.js', 'style.css'):
            raise HTTPException(404)
        return FileResponse(Path(__file__).parent / 'static' / name)

    @app.get('/api/state')
    def state():
        return store.snapshot() | dict(csrf=csrf, demo=demo, agents=[a.capability() for a in runtime.adapters.values()],
                                       limits=store.limits)

    @app.get('/api/events')
    async def events(request: Request, after: int = 0):
        async def stream():
            try:
                last = int(request.headers.get('last-event-id', '0'))
            except ValueError:
                last = 0
            cursor = max(0, after, last)
            while not await request.is_disconnected():
                # Ordered pagination avoids losing bursts of more than 200 events.
                with store.tx() as c:
                    rows = c.execute('SELECT * FROM events WHERE seq>? ORDER BY seq LIMIT 200', (cursor,)).fetchall()
                for row in rows:
                    e = dict(row)
                    e['data'] = json.loads(e['data'])
                    cursor = e['seq']
                    yield 'id: ' + str(cursor) + '\ndata: ' + json.dumps(e) + '\n\n'
                if not rows:
                    yield ': heartbeat\n\n'
                    await asyncio.sleep(1)
        return StreamingResponse(stream(), media_type='text/event-stream')

    async def body(request):
        try:
            data = await request.json()
        except (ValueError, UnicodeDecodeError):
            raise ValueError('Expected a JSON object') from None
        if not isinstance(data, dict):
            raise ValueError('Expected a JSON object')
        for key in ('agent', 'goal_id', 'title', 'instruction'):
            if key in data and (not isinstance(data[key], str) or len(data[key]) > 10000):
                raise ValueError('Invalid text field: ' + key)
        for key in ('brain_context', 'dependencies'):
            if key in data and (not isinstance(data[key], list) or len(data[key]) > 100 or any(not isinstance(x, str) for x in data[key])):
                raise ValueError('Invalid list field: ' + key)
        return data

    def agent(id):
        if not isinstance(id, str) or id not in runtime.adapters:
            raise ValueError('Unknown agent')
        return id

    @app.post('/api/goals')
    async def goal(request: Request):
        data = await body(request)
        context = data.get('brain_context', [])
        runtime.brain.select(context)
        created = store.create_goal(data.get('title'), agent(data.get('agent')), context)
        store.mode('running')
        return created

    @app.post('/api/tasks')
    async def task(request: Request):
        d = await body(request)
        runtime.brain.select(d.get('brain_context', []))
        if not isinstance(d.get('title'), str) or not 1 <= len(d['title']) <= 10000:
            raise ValueError('Task title required')
        return store.create_task(d.get('goal_id'), d['title'], agent(d.get('agent')), d.get('dependencies', []), d.get('brain_context', []))

    @app.post('/api/team/{mode}')
    def mode(mode: str):
        store.mode(mode)
        return {'mode': mode}

    @app.post('/api/tasks/{id}/{action}')
    async def task_control(id: str, action: str, request: Request):
        d = await body(request)
        if d.get('agent'):
            agent(d['agent'])
        store.control(id, action, d.get('instruction', ''), d.get('agent'))
        return {'ok': True}

    @app.post('/api/agents/{id}/stop')
    async def stop(id: str):
        return {'signalled': runtime.stop_agent(agent(id))}

    @app.post('/api/goals/{id}/instruction')
    async def intervene(id: str, request: Request):
        d = await body(request)
        instruction = d.get('instruction')
        if not isinstance(instruction, str) or not 1 <= len(instruction) <= 10000:
            raise ValueError('Instruction required')
        with store.tx() as c:
            g = store.get(c, 'goals', id)
            g['interventions'].append(clean(instruction))
            store.put(c, 'goals', g)
            store.event(c, 'HUMAN_INTERVENTION', id, instruction=instruction, delivery='next invocation')
        return {'delivery': 'next invocation; active work is not interrupted'}

    @app.get('/api/brain')
    def brain():
        return {'documents': runtime.brain.listing()}

    @app.get('/api/brain/read')
    def brain_read(path: str):
        return clean(runtime.brain.select([path])[0])
    return app
