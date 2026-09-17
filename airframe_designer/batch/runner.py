"""Run many headless simulations in parallel. Each worker process owns one PX4 SITL instance at a time, taken
from a shared pool of instance numbers (1..9; instance 0 is left to an interactive session)."""
from __future__ import annotations

import multiprocessing as mp
import os
import queue
import time
import traceback
from typing import Callable

from ..px4.sitl import instance_is_free


def _worker_main(task_q, result_q, instance_q, common: dict) -> None:
    from .worker import run_once
    while True:
        try:
            task = task_q.get(timeout=1.0)
        except queue.Empty:
            continue
        if task is None:
            break
        inst = instance_q.get()
        try:
            kw = dict(common); kw.update(task.get("options", {}))
            res = run_once(task["airframe"], task["scenario"], variables=task.get("variables"), instance=inst,
                           task_id=task.get("id"), **kw)
        except Exception as e:
            res = {"id": task.get("id"), "ok": False, "status": "error", "failures": [f"{type(e).__name__}: {e}"],
                   "traceback": traceback.format_exc(), "metrics": {}, "variables": task.get("variables", {})}
        finally:
            instance_q.put(inst)
        result_q.put(res)


def run_many(tasks: list[dict], workers: int = 4, instances: list[int] | None = None, progress: Callable[[dict, int, int], None] | None = None,
             **common) -> list[dict]:
    """tasks: [{"id", "airframe": path|dict, "scenario": path|dict, "variables": {path: value}, "options": {...}}].
    Returns results in task order. ``common`` are keyword arguments for run_once (speed, rate, substeps, ...)."""
    if not tasks:
        return []
    if instances is None:
        instances = [i for i in range(1, 10) if instance_is_free(i)]
    workers = max(1, min(workers, len(instances), len(tasks)))
    if workers == 1 and len(tasks) == 1:
        from .worker import run_once
        t = tasks[0]
        kw = dict(common); kw.update(t.get("options", {}))
        r = run_once(t["airframe"], t["scenario"], variables=t.get("variables"), instance=instances[0], task_id=t.get("id"), **kw)
        if progress:
            progress(r, 1, 1)
        return [r]
    ctx = mp.get_context("spawn")
    task_q, result_q, inst_q = ctx.Queue(), ctx.Queue(), ctx.Queue()
    for i in instances[:workers]:
        inst_q.put(i)
    procs = [ctx.Process(target=_worker_main, args=(task_q, result_q, inst_q, common), daemon=True) for _ in range(workers)]
    for p in procs:
        p.start()
    for k, t in enumerate(tasks):
        t = dict(t); t.setdefault("id", f"task{k}"); t["_k"] = k
        task_q.put(t)
    results: dict[int, dict] = {}
    by_id = {t.get("id", f"task{k}"): k for k, t in enumerate(tasks)}
    done = 0
    try:
        while done < len(tasks):
            r = result_q.get()
            k = by_id.get(r.get("id"), done)
            results[k] = r
            done += 1
            if progress:
                progress(r, done, len(tasks))
    finally:
        for _ in procs:
            task_q.put(None)
        for p in procs:
            p.join(5)
            if p.is_alive():
                p.terminate()
    return [results[k] for k in range(len(tasks))]
