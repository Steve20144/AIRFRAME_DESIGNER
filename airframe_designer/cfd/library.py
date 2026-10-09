"""The attitude library: a grid of OpenFOAM cases (alpha x beta) on one prepared surface, run in the background,
and the queries the CFD tab makes against it while the user turns the aircraft in the 3D view.

Layout under results/cfd/<surface name>/
  surface/            surface.stl, display.json, meta.json         (cfd.geometry.prepare_surface)
  lib_<kmh>kmh_<quality>/
     mesh/            the shared snappyHexMesh mesh (one per library; only the far-field velocity changes per case)
     cases/a+03.0_b+05.0/   result.json, surface_p.npz, cp_display.npy, logs
     cp_map.npy       display vertex -> nearest wall face (shared mesh, so shared by every case)
     library.json     grid, settings, status

Symmetry: the aircraft is mirror-symmetric in y, so only beta >= 0 is computed; a negative beta mirrors the
result (Fy, Mx, Mz change sign; the Cp field is read at the mirrored vertex).

Interpolation between the computed attitudes is linear on the Delaunay triangulation of the (alpha, beta) points,
with nearest-point fallback outside the hull. The same weights serve forces, moments and the Cp field, so what the
user sees and the numbers next to it always agree.
"""
from __future__ import annotations

import json
import math
import os
import re
import signal
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np

from . import case as C
from .geometry import prepare_surface, PROJECT_DIR

CFD_ROOT = PROJECT_DIR / "results" / "cfd"
G = 9.80665


def case_key(alpha: float, beta: float) -> str:
    return f"a{alpha:+05.1f}_b{beta:+05.1f}"


def default_nproc() -> int:
    n = os.cpu_count() or 2
    return max(1, min(8, n - 2))


# ----------------------------------------------------------------------------------------------- surfaces

def list_surfaces() -> list[dict]:
    out = []
    if not CFD_ROOT.exists():
        return out
    for d in sorted(CFD_ROOT.iterdir()):
        meta = d / "surface" / "meta.json"
        if meta.exists():
            m = json.loads(meta.read_text())
            libs = []
            for ld in sorted(d.glob("lib_*")):
                lj = ld / "library.json"
                if lj.exists():
                    try:
                        s = json.loads(lj.read_text())
                        libs.append({"name": ld.name, "speed_kmh": s.get("speed_kmh"), "quality": s.get("quality"),
                                     "done": sum(1 for c in s.get("cases", []) if c.get("status") == "done"), "total": len(s.get("cases", []))})
                    except Exception:  # noqa: BLE001
                        pass
            out.append({"name": d.name, "source": m.get("source"), "prepared_at": m.get("prepared_at"), "area_m2": m.get("area_m2"),
                        "span_m": m.get("span_m"), "length_m": m.get("length_m"), "cfd_tris": m.get("cfd_tris"), "libraries": libs})
    return out


def surface_name_for(source: Path) -> str:
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", Path(source).stem)
    return stem or "surface"


# ----------------------------------------------------------------------------------------------- processes

def _proc_alive(pid: int) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except (ProcessLookupError, ValueError):
        return False
    except PermissionError:
        return True


def _proc_tree(pid: int) -> list[int]:
    """pid and all its descendants (pgrep -P), so a pause / stop reaches mpirun and the solver ranks."""
    import subprocess
    out, todo = [], [int(pid)]
    while todo:
        p = todo.pop()
        out.append(p)
        try:
            r = subprocess.run(["pgrep", "-P", str(p)], capture_output=True, text=True, timeout=5)
            todo += [int(x) for x in r.stdout.split() if x.strip()]
        except Exception:  # noqa: BLE001
            pass
    return out


def _signal_tree(pid: int, sig) -> int:
    n = 0
    for p in reversed(_proc_tree(pid)):      # children first
        try:
            os.kill(p, sig)
            n += 1
        except Exception:  # noqa: BLE001
            pass
    return n


def _proc_state(pid: int) -> str | None:
    """'running', 'paused' (stopped by SIGSTOP) or None when gone (ps state letter on macOS / Linux)."""
    import subprocess
    try:
        r = subprocess.run(["ps", "-o", "stat=", "-p", str(pid)], capture_output=True, text=True, timeout=5)
        st = r.stdout.strip()
    except Exception:  # noqa: BLE001
        return None
    if not st:
        return None
    return "paused" if st[0] == "T" else "running"


# ----------------------------------------------------------------------------------------------- library

class Library:
    """One grid of cases on one surface. Thread-safe enough for one runner thread and HTTP readers."""

    def __init__(self, surface_dir: Path, speed_kmh: float, quality: str):
        self.surface_dir = Path(surface_dir)
        self.meta = json.loads((self.surface_dir / "meta.json").read_text())
        self.speed_kmh = float(speed_kmh)
        self.quality = quality
        self.dir = self.surface_dir.parent / f"lib_{int(round(speed_kmh))}kmh_{quality}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.thread: threading.Thread | None = None
        self.stop_flag = False
        self.log_lines: list[str] = []
        self.stage = ""
        self.current: str | None = None
        self.started_at = None
        self.error = None
        self.results: dict[str, dict] = {}
        self._cp_cache: dict[str, np.ndarray] = {}
        self._display = None
        self._cp_map = None
        self._mirror_map = None
        self._interp_cache = None
        self.grid = {"alphas": [], "betas": []}
        self.nproc = default_nproc()
        self.failed = {}
        self._loaded_mtime = {}
        self._flow_cache = {}
        self._load()

    # ---- persistence
    def _load(self):
        lj = self.dir / "library.json"
        if lj.exists():
            try:
                s = json.loads(lj.read_text())
                self.grid = s.get("grid", self.grid)
                self.nproc = s.get("nproc", self.nproc)
            except Exception:  # noqa: BLE001
                pass
        for rj in (self.dir / "cases").glob("*/result.json"):
            try:
                r = json.loads(rj.read_text())
                self.results[rj.parent.name] = r
            except Exception:  # noqa: BLE001
                pass
        self._interp_cache = None

    def _save(self):
        s = {"speed_kmh": self.speed_kmh, "quality": self.quality, "grid": self.grid, "nproc": self.nproc,
             "surface": self.meta.get("source"), "surface_hash": self.meta.get("source_hash"),
             "updated_at": datetime.now().astimezone().isoformat(timespec="seconds"), "cases": self.case_list()}
        (self.dir / "library.json").write_text(json.dumps(s, indent=1))

    def log(self, msg: str):
        line = f"{datetime.now().strftime('%H:%M:%S')} {msg}"
        self.log_lines.append(line)
        del self.log_lines[:-400]
        try:
            with open(self.dir / "runner.log", "a") as f:
                f.write(line + "\n")
        except OSError:
            pass

    def runner_log_tail(self, n: int = 40) -> list[str]:
        f = self.dir / "runner.log"
        if not f.exists():
            return []
        try:
            lines = f.read_text().splitlines()
        except OSError:
            return []
        return lines[-n:]

    # ---- the runner process (the library always computes in a separate process the app watches and controls)
    @property
    def runner_file(self) -> Path:
        return self.dir / "runner.json"

    def runner(self) -> dict | None:
        """{pid, cmd, started_at, state: running|paused} of the worker process, or None (stale files are removed)."""
        f = self.runner_file
        if not f.exists():
            return None
        try:
            r = json.loads(f.read_text())
        except Exception:  # noqa: BLE001
            return None
        st = _proc_state(int(r.get("pid", -1)))
        if st is None:
            try:
                f.unlink()
            except OSError:
                pass
            return None
        r["state"] = st
        return r

    def write_runner(self, pid: int, cmd: str):
        self.runner_file.write_text(json.dumps({"pid": int(pid), "cmd": cmd, "started_at": datetime.now().astimezone().isoformat(timespec="seconds")}))

    def clear_runner(self):
        try:
            self.runner_file.unlink()
        except OSError:
            pass

    @property
    def running(self) -> bool:
        if self.thread is not None and self.thread.is_alive():
            return True
        return self.runner() is not None

    def pause(self) -> bool:
        r = self.runner()
        if not r:
            return False
        _signal_tree(r["pid"], signal.SIGSTOP)
        return True

    def resume(self) -> bool:
        r = self.runner()
        if not r:
            return False
        _signal_tree(r["pid"], signal.SIGCONT)
        return True

    def spawn(self, alphas, betas, nproc: int | None = None, one: tuple[float, float] | None = None) -> dict:
        """Start the worker process (this module's CLI) for the grid, or for one extra attitude."""
        import subprocess
        import sys
        if self.running:
            raise RuntimeError("the library is already being computed")
        if nproc:
            self.nproc = int(nproc)
        if not one:
            self.grid = {"alphas": sorted(set(round(float(a), 1) for a in alphas)), "betas": sorted(set(round(abs(float(b)), 1) for b in betas))}
        self.failed = {}
        self._save()
        cmd = [sys.executable, "-m", "airframe_designer.cfd.library", self.surface_dir.parent.name, "--speed-kmh", str(self.speed_kmh),
               "--quality", self.quality, "--nproc", str(self.nproc), "--worker"]
        if one:
            cmd += ["--one", f"{one[0]},{one[1]}"]
        else:
            cmd += ["--alphas", ",".join(str(a) for a in self.grid["alphas"]), "--betas", ",".join(str(b) for b in self.grid["betas"])]
        logf = open(self.dir / "runner.log", "a")
        p = subprocess.Popen(cmd, cwd=str(PROJECT_DIR), stdin=subprocess.DEVNULL, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
        self.write_runner(p.pid, " ".join(cmd[2:]))
        self.log(f"worker started (pid {p.pid})" + (f" for one attitude {one}" if one else ""))
        return {"pid": p.pid}

    def mesh_info(self) -> dict | None:
        mj = self.dir / "mesh" / "mesh.json"
        return json.loads(mj.read_text()) if mj.exists() else None

    def planned_keys(self) -> list[str]:
        keys = []
        for b in sorted(set(abs(float(x)) for x in self.grid.get("betas", []))):
            for a in sorted(set(float(x) for x in self.grid.get("alphas", []))):
                keys.append(case_key(a, b))
        return keys

    def case_list(self) -> list[dict]:
        out = []
        keys = self.planned_keys()
        for k in keys + [k for k in self.results if k not in keys]:
            r = self.results.get(k)
            m = re.match(r"a([+-]\d+\.\d)_b([+-]\d+\.\d)", k)
            a, b = (float(m.group(1)), float(m.group(2))) if m else (None, None)
            row = {"key": k, "alpha": a, "beta": b, "status": "pending" if k in keys else "extra"}
            if r:
                row.update({"status": "done" if r.get("ok", True) else "doubtful", "lift": r["lift"], "drag": r["drag"], "side": r["side"],
                            "My0": r["M0_frd"][1], "iterations": r["iterations"], "elapsed_s": r["elapsed_s"], "scatter": r.get("force_scatter")})
            elif (k == self.current and self.running) or ((self.dir / "cases" / k / "log.simpleFoam").exists() and not (self.dir / "cases" / k / "result.json").exists()):
                row["status"] = "running"
            elif k in self.failed:
                row["status"] = "failed"
            out.append(row)
        return out

    failed: dict[str, str] = {}

    def _rescan(self):
        """Pick up result.json files written by another process (the headless CLI) while this instance is idle."""
        if self.running:
            return
        for rj in (self.dir / "cases").glob("*/result.json"):
            k = rj.parent.name
            try:
                mt = rj.stat().st_mtime
            except OSError:
                continue
            if k not in self.results or self._loaded_mtime.get(k) != mt:
                try:
                    with self.lock:
                        self.results[k] = json.loads(rj.read_text())
                        self._cp_cache.pop(k, None)
                        self._interp_cache = None
                    self._loaded_mtime[k] = mt
                except Exception:  # noqa: BLE001
                    pass

    _loaded_mtime: dict[str, float] = {}

    def live_progress(self) -> dict | None:
        """The attitude being computed right now (by this process or a headless one): iteration count and the lift
        history so far, read from the case directory."""
        cdir = self.dir / "cases"
        if not cdir.exists():
            return None
        running = []
        for d in cdir.iterdir():
            if d.is_dir() and not (d / "result.json").exists() and (d / "log.simpleFoam").exists():
                running.append(d)
        if not running:
            if self.running and (self.stage.startswith("snappy") or self.stage.startswith("blockMesh") or self.stage == "meshing"):
                return {"key": None, "stage": self.stage, "meshing": True}
            return None
        d = max(running, key=lambda p: (p / "log.simpleFoam").stat().st_mtime)
        age = time.time() - (d / "log.simpleFoam").stat().st_mtime
        try:
            it_total = int(re.search(r"endTime\s+(\d+);", (d / "system" / "controlDict").read_text()).group(1))
        except Exception:  # noqa: BLE001
            it_total = None
        hist = {"it": [], "lift": [], "drag": []}
        it = 0
        m = re.match(r"a([+-]\d+\.\d)_b([+-]\d+\.\d)", d.name)
        a, b = (float(m.group(1)), float(m.group(2))) if m else (0.0, 0.0)
        fdir = d / "postProcessing" / "forces"
        for sub in sorted(fdir.glob("*")) if fdir.exists() else []:
            fp = sub / "force.dat"
            if fp.exists():
                try:
                    with open(fp) as f:
                        for line in f:
                            if line.startswith("#") or not line.strip():
                                continue
                            nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line.replace("(", " ").replace(")", " "))]
                            if len(nums) >= 4:
                                F = np.array(nums[1:4])
                                wa = C.wind_axes(F, a, b)
                                hist["it"].append(int(nums[0])); hist["lift"].append(wa["lift"]); hist["drag"].append(wa["drag"])
                except Exception:  # noqa: BLE001
                    pass
        if hist["it"]:
            it = hist["it"][-1]
        # thin the history for the browser
        if len(hist["it"]) > 400:
            step = len(hist["it"]) // 400 + 1
            hist = {k: v[::step] for k, v in hist.items()}
        return {"key": d.name, "alpha": a, "beta": b, "iteration": it, "iterations": it_total, "history": hist,
                "stale": age > 120, "age_s": round(age), "stage": self.stage if self.running else None}

    def status(self) -> dict:
        self._rescan()
        cases = self.case_list()
        done = [c for c in cases if c["status"] in ("done", "doubtful")]
        planned = [c for c in cases if c["status"] != "extra"]
        remaining = [c for c in planned if c["status"] in ("pending", "running")]
        mean_t = float(np.mean([c["elapsed_s"] for c in done])) if done else None
        return {
            "dir": str(self.dir.relative_to(PROJECT_DIR)), "speed_kmh": self.speed_kmh, "quality": self.quality, "nproc": self.nproc,
            "running": self.running, "stage": self.stage, "current": self.current, "error": self.error, "started_at": self.started_at,
            "grid": self.grid, "cases": cases, "done": len(done), "total": len(planned), "mesh": self.mesh_info(),
            "eta_s": (mean_t * len(remaining)) if (mean_t and remaining) else None, "log": self.runner_log_tail(40),
            "live": self.live_progress(), "runner": self.runner(),
        }

    # ---- running
    def start(self, alphas: list[float], betas: list[float], nproc: int | None = None, rebuild_mesh: bool = False):
        """From the app: compute in a worker process (pause / resume / stop from the jobs panel)."""
        return self.spawn(alphas, betas, nproc)

    def start_thread(self, alphas: list[float], betas: list[float], nproc: int | None = None, rebuild_mesh: bool = False):
        """In this process (the worker CLI uses it)."""
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("library already running")
        self.grid = {"alphas": sorted(set(round(float(a), 1) for a in alphas)), "betas": sorted(set(round(abs(float(b)), 1) for b in betas))}
        if nproc:
            self.nproc = int(nproc)
        self.stop_flag = False
        self.error = None
        self.failed = {}
        self.started_at = datetime.now().astimezone().isoformat(timespec="seconds")
        self._save()
        self.thread = threading.Thread(target=self._run, args=(rebuild_mesh,), daemon=True, name="cfd-library")
        self.thread.start()

    def stop(self):
        self.stop_flag = True
        C.kill_running()
        r = self.runner()
        if r:
            _signal_tree(r["pid"], signal.SIGCONT)      # a paused tree must wake up to die
            _signal_tree(r["pid"], signal.SIGTERM)
            self.log(f"worker {r['pid']} stopped by the user")
            # the attitude that was being computed is incomplete: remove its directory so it is redone later
            for d in (self.dir / "cases").glob("*"):
                if d.is_dir() and not (d / "result.json").exists():
                    import shutil
                    shutil.rmtree(d, ignore_errors=True)
            self.clear_runner()

    def _run(self, rebuild_mesh: bool):
        try:
            self.log(f"computing {len(self.planned_keys())} attitudes on {self.nproc} cores")
            self._ensure_mesh(rebuild_mesh)
            for key in self.planned_keys():
                if self.stop_flag:
                    self.log("stopped by the user")
                    break
                if key in self.results and (self.dir / "cases" / key / "flow.npz").exists():
                    continue
                if key in self.results:
                    self.log(f"{key}: no flow field yet, recomputing")
                    self.results.pop(key, None)
                self._run_key(key)
        except Exception as e:  # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            self.log("ERROR " + self.error)
            self.log(traceback.format_exc().splitlines()[-1])
        finally:
            self.current = None
            self.stage = "idle"
            self._save()

    def _ensure_mesh(self, rebuild: bool):
        mesh_dir = self.dir / "mesh"
        if rebuild or not (mesh_dir / "constant" / "polyMesh" / "points").exists():
            self.stage = "meshing"
            self.log(f"building the {self.quality} mesh on {self.nproc} cores")
            C.build_mesh(mesh_dir, self.surface_dir / "surface.stl", self.meta["bounds_frd"], self.quality, self.nproc,
                         log=self.log, progress=lambda s: setattr(self, "stage", s))
            self._cp_map = None
            for p in self.dir.glob("cp_map*.npy"):
                p.unlink()

    def run_one(self, alpha: float, beta: float, nproc: int | None = None):
        """One extra attitude outside the grid (or a redo), in a worker process."""
        return self.spawn(None, None, nproc, one=(round(float(alpha), 1), round(abs(float(beta)), 1)))

    def run_one_thread(self, alpha: float, beta: float, nproc: int | None = None):
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("library already running")
        if nproc:
            self.nproc = int(nproc)
        self.stop_flag = False
        self.error = None
        key = case_key(round(float(alpha), 1), round(abs(float(beta)), 1))
        self.results.pop(key, None)

        def go():
            try:
                self._ensure_mesh(False)
                self._run_key(key)
            except Exception as e:  # noqa: BLE001
                self.error = f"{type(e).__name__}: {e}"
                self.log("ERROR " + self.error)
            finally:
                self.current = None
                self.stage = "idle"
                self._save()
        self.thread = threading.Thread(target=go, daemon=True, name="cfd-one")
        self.thread.start()

    def _run_key(self, key: str):
        m = re.match(r"a([+-]\d+\.\d)_b([+-]\d+\.\d)", key)
        a, b = float(m.group(1)), float(m.group(2))
        self.current = key
        self.stage = f"{key}: starting"
        try:
            r = C.run_case(self.dir / "cases" / key, self.dir / "mesh", a, b, self.speed_kmh / 3.6, self.quality, self.nproc,
                           log=self.log, progress=lambda s: setattr(self, "stage", f"{key}: {s}"))
        except Exception as e:  # noqa: BLE001
            if self.stop_flag:
                return
            self.failed[key] = str(e)
            self.log(f"{key} FAILED: {e}")
            return
        with self.lock:
            self.results[key] = r
            self._cp_cache.pop(key, None)
            self._interp_cache = None
        try:
            self._cp_for(key)      # map the wall pressures onto the display mesh now, while the case is fresh
        except Exception as e:  # noqa: BLE001
            self.log(f"{key}: Cp mapping failed: {e}")
        self._save()

    # ---- display mesh and Cp fields
    def display(self) -> dict:
        if self._display is None:
            self._display = json.loads((self.surface_dir / "display.json").read_text())
        return self._display

    def _display_vertices(self) -> np.ndarray:
        return np.asarray(self.display()["vertices"], np.float64).reshape(-1, 3)

    def _mirror(self) -> np.ndarray:
        """index of the display vertex nearest the y-mirror of each vertex"""
        if self._mirror_map is None:
            from scipy.spatial import cKDTree
            v = self._display_vertices()
            t = cKDTree(v)
            _, idx = t.query(v * np.array([1.0, -1.0, 1.0]))
            self._mirror_map = idx
        return self._mirror_map

    def _cp_for(self, key: str) -> np.ndarray | None:
        """Cp per display vertex (float32) for a computed case (cached on disk next to the case)."""
        if key in self._cp_cache:
            return self._cp_cache[key]
        cdir = self.dir / "cases" / key
        f = cdir / "cp_display.npy"
        if f.exists():
            arr = np.load(f)
        else:
            npz = cdir / "surface_p.npz"
            if not npz.exists():
                return None
            d = np.load(npz)
            centres, p = d["centres"], d["p"]
            V = self.results[key]["speed_ms"]
            cp_face = p / (0.5 * V * V)
            mp = self.dir / "cp_map.npy"
            if self._cp_map is None:
                if mp.exists():
                    self._cp_map = np.load(mp)
                if self._cp_map is None or len(self._cp_map) != len(self._display_vertices()):
                    from scipy.spatial import cKDTree
                    _, idx = cKDTree(centres).query(self._display_vertices())
                    self._cp_map = idx
                    np.save(mp, idx)
            if self._cp_map.max() >= len(cp_face):      # the mesh changed under us
                from scipy.spatial import cKDTree
                _, self._cp_map = cKDTree(centres).query(self._display_vertices())
                np.save(mp, self._cp_map)
            arr = cp_face[self._cp_map].astype(np.float32)
            np.save(f, arr)
        self._cp_cache[key] = arr
        return arr

    # ---- interpolation
    def _points(self):
        """(keys, points Nx2 (alpha, beta), mirrored flags) over the computed cases, beta mirrored to negative too."""
        self._rescan()
        with self.lock:
            keys = [k for k, r in self.results.items() if np.all(np.isfinite(r["F_frd"]))]
        pts, flags, ks = [], [], []
        for k in keys:
            r = self.results[k]
            a, b = r["alpha_deg"], r["beta_deg"]
            pts.append((a, b)); flags.append(False); ks.append(k)
            if abs(b) > 1e-9:
                pts.append((a, -b)); flags.append(True); ks.append(k)
        return ks, np.array(pts, float).reshape(-1, 2), flags

    def weights(self, alpha: float, beta: float) -> list[tuple[str, float, bool]]:
        """[(case key, weight, mirrored)] for a linear interpolation at (alpha, beta)."""
        ks, P, flags = self._points()
        if len(ks) == 0:
            return []
        if len(ks) == 1:
            return [(ks[0], 1.0, flags[0])]
        a = float(np.clip(alpha, P[:, 0].min(), P[:, 0].max()))
        b = float(np.clip(beta, P[:, 1].min(), P[:, 1].max()))
        if np.ptp(P[:, 1]) < 1e-9 or np.ptp(P[:, 0]) < 1e-9:       # one line of points: 1-D interpolation
            axis = 0 if np.ptp(P[:, 1]) < 1e-9 else 1
            x = P[:, axis]
            xq = a if axis == 0 else b
            order = np.argsort(x)
            xs = x[order]
            i = int(np.searchsorted(xs, xq))
            if i <= 0:
                return [(ks[order[0]], 1.0, flags[order[0]])]
            if i >= len(xs):
                return [(ks[order[-1]], 1.0, flags[order[-1]])]
            t = (xq - xs[i - 1]) / max(xs[i] - xs[i - 1], 1e-12)
            return [(ks[order[i - 1]], 1.0 - t, flags[order[i - 1]]), (ks[order[i]], t, flags[order[i]])]
        from scipy.spatial import Delaunay, cKDTree
        if self._interp_cache is None or self._interp_cache[0] != len(ks):
            try:
                tri = Delaunay(P)
            except Exception:  # noqa: BLE001
                tri = None
            self._interp_cache = (len(ks), tri, cKDTree(P))
        _, tri, tree = self._interp_cache
        if tri is not None:
            s = tri.find_simplex(np.array([[a, b]]))[0]
            if s >= 0:
                T = tri.transform[s]
                bc = T[:2].dot(np.array([a, b]) - T[2])
                w = np.append(bc, 1.0 - bc.sum())
                return [(ks[v], float(wi), flags[v]) for v, wi in zip(tri.simplices[s], w) if wi > 1e-9]
        _, i = tree.query([a, b])
        return [(ks[i], 1.0, flags[i])]

    def query(self, alpha: float, beta: float, mass_kg: float | None, cg: list[float] | None, want_cp: bool = True) -> dict:
        """Interpolated forces/moments/coefficients at (alpha, beta), moments about the CG, Cp per display vertex."""
        w = self.weights(alpha, beta)
        if not w:
            return {"ok": False, "error": "no computed attitude yet"}
        F = np.zeros(3); M0 = np.zeros(3)
        cp = None
        for key, wi, mirrored in w:
            r = self.results[key]
            f = np.array(r["F_frd"]); m = np.array(r["M0_frd"])
            if mirrored:
                f = f * np.array([1, -1, 1]); m = m * np.array([-1, 1, -1])
            F += wi * f; M0 += wi * m
            if want_cp:
                c = self._cp_for(key)
                if c is not None:
                    if mirrored:
                        c = c[self._mirror()]
                    cp = (wi * c) if cp is None else cp + wi * c
        V = self.speed_kmh / 3.6
        q = 0.5 * C.RHO * V * V
        S = self.meta["area_m2"]; cbar = self.meta["mean_chord_m"]; b = self.meta["span_m"]
        cg = np.array(cg if cg is not None else [0.0, 0.0, 0.0], float)
        Mcg = M0 - np.cross(cg, F)
        wa = C.wind_axes(F, alpha, beta)
        out = {
            "ok": True, "alpha_deg": alpha, "beta_deg": beta, "speed_kmh": self.speed_kmh,
            "F_frd": F.tolist(), "M_cg_frd": Mcg.tolist(), "M0_frd": M0.tolist(), **wa,
            "CL": wa["lift"] / (q * S), "CD": wa["drag"] / (q * S), "CY": wa["side"] / (q * S),
            "Cl": Mcg[0] / (q * S * b), "Cm": Mcg[1] / (q * S * cbar), "Cn": Mcg[2] / (q * S * b),
            "weights": [{"key": k, "w": round(wi, 4), "mirrored": mi} for k, wi, mi in w],
            "exact": len(w) == 1 and abs(w[0][1] - 1.0) < 1e-9 and abs(self.results[w[0][0]]["alpha_deg"] - alpha) < 1e-6
                     and abs(abs(self.results[w[0][0]]["beta_deg"]) - abs(beta)) < 1e-6,
            "ref": {"S_m2": S, "c_m": cbar, "b_m": b, "q_Pa": q},
        }
        if mass_kg:
            out["weight_N"] = mass_kg * G
            out["L_over_W"] = wa["lift"] / (mass_kg * G)
            out["thrust_needed_N"] = wa["drag"]
        if cp is not None:
            out["cp"] = cp.astype(np.float32)
        return out

    # ---- flow field for the animation
    def _flow_for(self, key: str):
        if key in self._flow_cache:
            return self._flow_cache[key]
        f = self.dir / "cases" / key / "flow.npz"
        if not f.exists():
            return None
        d = np.load(f)
        fl = {"origin": d["origin"], "spacing": float(d["spacing"]), "shape": d["shape"], "U": d["U"], "p": d["p"]}
        if len(self._flow_cache) > 6:
            self._flow_cache.pop(next(iter(self._flow_cache)))
        self._flow_cache[key] = fl
        return fl

    _flow_cache: dict = {}

    def flow(self, alpha: float, beta: float) -> dict | None:
        """Interpolated velocity (m/s, body axes) and Cp on the sampling grid at (alpha, beta); None without data."""
        w = self.weights(alpha, beta)
        if not w:
            return None
        U = None; P = None; meta = None; wsum = 0.0; used = []
        V = self.speed_kmh / 3.6
        for key, wi, mirrored in w:
            fl = self._flow_for(key)
            if fl is None:
                continue
            u = fl["U"]; p = fl["p"]
            if mirrored:
                u = u[:, ::-1, :, :] * np.array([1, -1, 1], np.float32)
                p = p[:, ::-1, :]
            if U is None:
                U = wi * u; P = wi * p; meta = fl
            else:
                if u.shape != U.shape:
                    continue
                U = U + wi * u; P = P + wi * p
            wsum += wi; used.append(key)
        if U is None:
            return None
        if wsum > 0 and abs(wsum - 1.0) > 1e-6:      # some weights had no flow data: renormalise what we have
            U = U / wsum; P = P / wsum
        return {"origin": meta["origin"].tolist(), "spacing": meta["spacing"], "shape": [int(x) for x in meta["shape"]],
                "speed_ms": V, "U": U.astype(np.float32), "cp": (P / (0.5 * V * V)).astype(np.float32), "cases": used}

    # ---- stability
    def stability(self, mass_kg: float | None, cg: list[float] | None) -> dict:
        """Static derivatives, trim and plain-language verdicts from the computed grid (no Cp)."""
        with self.lock:
            done = {k: r for k, r in self.results.items() if np.all(np.isfinite(r["F_frd"]))}
        if not done:
            return {"ok": False, "error": "no computed attitude yet"}
        alphas = sorted(set(round(r["alpha_deg"], 3) for r in done.values()))
        betas = sorted(set(round(abs(r["beta_deg"]), 3) for r in done.values()))
        V = self.speed_kmh / 3.6
        q = 0.5 * C.RHO * V * V
        S = self.meta["area_m2"]; cbar = self.meta["mean_chord_m"]; bspan = self.meta["span_m"]
        rows = []
        for a in alphas:
            r = self.query(a, 0.0, mass_kg, cg, want_cp=False)
            if r.get("ok") and r["weights"] and r["weights"][0]["w"] > 0.999 and abs(self.results[r["weights"][0]["key"]]["beta_deg"]) < 1e-9:
                rows.append({"alpha": a, "CL": r["CL"], "CD": r["CD"], "Cm": r["Cm"], "lift_N": r["lift"], "drag_N": r["drag"], "My_cg": r["M_cg_frd"][1],
                             "L_over_W": r.get("L_over_W")})
        out = {"ok": True, "speed_kmh": self.speed_kmh, "ref": {"S_m2": S, "c_m": cbar, "b_m": bspan}, "mass_kg": mass_kg, "cg": cg,
               "longitudinal": rows, "lateral": [], "verdicts": [], "notes": []}
        W = mass_kg * G if mass_kg else None
        if len(rows) >= 2:
            A = np.array([r["alpha"] for r in rows]); CL = np.array([r["CL"] for r in rows]); Cm = np.array([r["Cm"] for r in rows])
            # slopes from a straight-line fit through the computed points (per degree)
            cma = float(np.polyfit(A, Cm, 1)[0]); cla = float(np.polyfit(A, CL, 1)[0])
            sm = (-cma / cla) if cla > 1e-6 else None      # meaningless unless lift grows with alpha
            if cla <= 1e-6:
                out["notes"].append("lift does not grow with alpha over the computed points: static margin not defined (results not converged?)")
            out["Cm_alpha_per_deg"] = cma; out["CL_alpha_per_deg"] = cla
            out["static_margin"] = sm
            if sm is not None and cg is not None:
                out["neutral_point_x"] = float(cg[0] - sm * cbar)
            # trim: Cm = 0 crossing
            trim = None
            for i in range(len(A) - 1):
                if Cm[i] == 0 or Cm[i] * Cm[i + 1] < 0:
                    t = Cm[i] / (Cm[i] - Cm[i + 1]) if Cm[i] != Cm[i + 1] else 0.0
                    trim = {"alpha": float(A[i] + t * (A[i + 1] - A[i])), "CL": float(CL[i] + t * (CL[i + 1] - CL[i]))}
                    break
            if trim:
                trim["lift_N"] = trim["CL"] * q * S
                if W:
                    trim["L_over_W"] = trim["lift_N"] / W
                    trim["speed_for_weight_kmh"] = math.sqrt(2 * W / (C.RHO * S * trim["CL"])) * 3.6 if trim["CL"] > 0 else None
            out["trim"] = trim
            # alpha that carries the weight at this speed, and the pitching moment left there
            if W:
                CLreq = W / (q * S)
                out["CL_required"] = CLreq
                if CL.min() <= CLreq <= CL.max():
                    aw = float(np.interp(CLreq, CL, A)) if np.all(np.diff(CL) > 0) else None
                    if aw is not None:
                        out["alpha_for_weight"] = aw
                        out["Cm_at_weight"] = float(np.interp(aw, A, Cm))
                        out["My_at_weight_Nm"] = out["Cm_at_weight"] * q * S * cbar
                else:
                    out["alpha_for_weight"] = None
            # verdicts
            if cma < -1e-4:
                smtxt = f", static margin {sm * 100:.0f} % of the mean chord" if sm is not None else ""
                out["verdicts"].append({"axis": "pitch", "ok": True, "text": f"Pitch: stable. A nose-up disturbance makes a nose-down moment (Cm_alpha {cma:+.4f}/deg{smtxt})."})
            elif cma > 1e-4:
                smtxt = f", static margin {sm * 100:.0f} %" if sm is not None else ""
                out["verdicts"].append({"axis": "pitch", "ok": False, "text": f"Pitch: unstable. A nose-up disturbance grows (Cm_alpha {cma:+.4f}/deg{smtxt}): it will not return on its own."})
            else:
                out["verdicts"].append({"axis": "pitch", "ok": None, "text": "Pitch: neutral, no restoring tendency."})
            if trim is None:
                out["verdicts"].append({"axis": "trim", "ok": False, "text": "No trim in the computed alpha range: the pitching moment never crosses zero, so something (elevon, CG move) must hold the nose."})
            elif W:
                out["verdicts"].append({"axis": "trim", "ok": trim["L_over_W"] >= 1.0,
                                        "text": f"Hands-off trim at alpha {trim['alpha']:+.1f} deg carries {trim['L_over_W'] * 100:.0f} % of the weight at {self.speed_kmh:.0f} km/h"
                                                + (f" (it would need {trim['speed_for_weight_kmh']:.0f} km/h to fly level)." if trim.get("speed_for_weight_kmh") else ".")})
        else:
            out["notes"].append("at least two alphas at beta 0 are needed for the pitch verdict")
        # lateral-directional at the alpha nearest the trim (or the middle one)
        if len(betas) >= 2:
            a_ref = out.get("trim", {}).get("alpha") if out.get("trim") else (out.get("alpha_for_weight") or alphas[len(alphas) // 2])
            a_ref = float(min(alphas, key=lambda x: abs(x - (a_ref if a_ref is not None else 0.0))))
            lat = []
            for b in betas:
                k = case_key(a_ref, b)
                if k in done:
                    r = self.query(a_ref, b, mass_kg, cg, want_cp=False)
                    lat.append({"beta": b, "CY": r["CY"], "Cl": r["Cl"], "Cn": r["Cn"]})
            out["lateral"] = lat
            out["lateral_alpha"] = a_ref
            if len(lat) >= 2:
                B = np.array([r["beta"] for r in lat])
                cnb = float(np.polyfit(B, [r["Cn"] for r in lat], 1)[0]); clb = float(np.polyfit(B, [r["Cl"] for r in lat], 1)[0])
                out["Cn_beta_per_deg"] = cnb; out["Cl_beta_per_deg"] = clb
                out["verdicts"].append({"axis": "yaw", "ok": cnb > 1e-5, "text": ("Yaw: weathercock stable, the nose turns back into the wind" if cnb > 1e-5 else "Yaw: no weathercock stability, a sideslip does not correct itself") + f" (Cn_beta {cnb:+.5f}/deg)."})
                out["verdicts"].append({"axis": "roll", "ok": clb < -1e-5, "text": ("Roll: a sideslip rolls the aircraft away from the wind (dihedral effect, stabilising)" if clb < -1e-5 else "Roll: no dihedral effect, a sideslip does not level the wings") + f" (Cl_beta {clb:+.5f}/deg)."})
        else:
            out["notes"].append("at least two betas are needed for the yaw and roll verdicts")
        out["notes"].append("steady RANS (k-omega SST), clean foil without fans or jets, no control surfaces; static tendencies only, not the damped motion")
        return out


# ----------------------------------------------------------------------------------------------- manager

class Manager:
    """What the server holds: the selected surface and its library (one at a time)."""

    def __init__(self):
        self.lib: Library | None = None
        self.prep_thread: threading.Thread | None = None
        self.prep_log: list[str] = []
        self.prep_error = None
        self.prep_result = None

    def select(self, surface_name: str, speed_kmh: float, quality: str) -> Library:
        sdir = CFD_ROOT / surface_name / "surface"
        if not (sdir / "meta.json").exists():
            raise FileNotFoundError(f"no prepared surface {surface_name}")
        if self.lib and self.lib.surface_dir == sdir and self.lib.speed_kmh == float(speed_kmh) and self.lib.quality == quality:
            return self.lib
        if self.lib and self.lib.running:
            raise RuntimeError("a library is running; stop it first")
        self.lib = Library(sdir, speed_kmh, quality)
        return self.lib

    def prepare(self, source: str, name: str | None = None, **kw):
        if self.prep_thread and self.prep_thread.is_alive():
            raise RuntimeError("a surface is already being prepared")
        src = Path(source).expanduser()
        if not src.is_absolute():
            src = (PROJECT_DIR / src).resolve()
        if not src.exists():
            raise FileNotFoundError(str(src))
        name = name or surface_name_for(src)
        out = CFD_ROOT / name / "surface"
        self.prep_log = []
        self.prep_error = None
        self.prep_result = None

        def go():
            try:
                self.prep_result = prepare_surface(src, out, log=self.prep_log.append, **kw)
                if self.lib and self.lib.surface_dir == out:
                    self.lib = None     # libraries of the old surface no longer match
            except Exception as e:  # noqa: BLE001
                self.prep_error = f"{type(e).__name__}: {e}"
                self.prep_log.append("ERROR " + self.prep_error)
        self.prep_thread = threading.Thread(target=go, daemon=True, name="cfd-prepare")
        self.prep_thread.start()
        return name

    def jobs(self) -> list[dict]:
        """Everything this app knows is computing in the background, one row each, for the jobs panel."""
        rows = []
        if self.prep_thread and self.prep_thread.is_alive():
            rows.append({"id": "prepare", "kind": "surface", "title": "Preparing the surface", "state": "running", "controls": ["none"],
                         "detail": (self.prep_log[-1].replace("[cfd.geometry] ", "") if self.prep_log else ""), "progress": None})
        libs = []
        if self.lib:
            libs.append(self.lib)
        # workers of other libraries (started earlier, or from the shell) show up too
        for rf in CFD_ROOT.glob("*/lib_*/runner.json"):
            if self.lib and rf.parent == self.lib.dir:
                continue
            try:
                other = Library(rf.parent.parent / "surface", float(rf.parent.name.split("_")[1].replace("kmh", "")), rf.parent.name.split("_")[2])
                libs.append(other)
            except Exception:  # noqa: BLE001
                pass
        for lib in libs:
            st = lib.status()
            r = st.get("runner")
            live = st.get("live") or {}
            if not r and not (lib.thread and lib.thread.is_alive()):
                continue
            state = r["state"] if r else "running"
            rows.append({
                "id": f"lib:{lib.dir.relative_to(CFD_ROOT)}", "kind": "library", "title": f"CFD library {lib.dir.parent.name} · {lib.speed_kmh:g} km/h · {lib.quality}",
                "state": state, "pid": r["pid"] if r else None, "started_at": r.get("started_at") if r else st.get("started_at"),
                "cores": lib.nproc, "controls": ["pause", "stop"] if state == "running" else ["resume", "stop"],
                "progress": {"done": st["done"], "total": st["total"], "eta_s": st.get("eta_s"),
                             "current": live.get("key"), "iteration": live.get("iteration"), "iterations": live.get("iterations"), "stage": live.get("stage") or st.get("stage")},
                "detail": (f"{live['key']}: iteration {live['iteration']} / {live['iterations']}" if live.get("key") else (live.get("stage") or "")),
            })
        return rows

    def job_action(self, job_id: str, action: str) -> dict:
        if job_id.startswith("lib:"):
            rel = job_id[4:]
            d = CFD_ROOT / rel
            lib = self.lib if (self.lib and self.lib.dir == d) else Library(d.parent / "surface", float(d.name.split("_")[1].replace("kmh", "")), d.name.split("_")[2])
            if action == "pause":
                ok = lib.pause()
            elif action == "resume":
                ok = lib.resume()
            elif action == "stop":
                lib.stop(); ok = True
            else:
                raise ValueError(f"unknown action {action}")
            return {"ok": ok}
        raise ValueError(f"no controls for {job_id}")

    def prepare_status(self) -> dict:
        return {"running": bool(self.prep_thread and self.prep_thread.is_alive()), "log": self.prep_log[-30:],
                "error": self.prep_error, "result": self.prep_result}


if __name__ == "__main__":
    import argparse
    import sys
    ap = argparse.ArgumentParser(description="Compute an attitude library (the app starts this as its worker process; it also runs by hand).")
    ap.add_argument("surface", help="surface name under results/cfd/")
    ap.add_argument("--speed-kmh", type=float, default=60.0)
    ap.add_argument("--quality", default="standard", choices=list(C.QUALITY))
    ap.add_argument("--alphas", default="-6,-3,0,3,6,9,12")
    ap.add_argument("--betas", default="0,5,10")
    ap.add_argument("--nproc", type=int, default=default_nproc())
    ap.add_argument("--one", default=None, help="alpha,beta: compute this one attitude instead of the grid")
    ap.add_argument("--worker", action="store_true", help="started by the app (quiet; same behaviour)")
    a = ap.parse_args()
    lib = Library(CFD_ROOT / a.surface / "surface", a.speed_kmh, a.quality)
    if lib.runner() is not None:
        print(f"a worker is already computing this library (pid {lib.runner()['pid']})", file=sys.stderr)
        sys.exit(1)
    lib.write_runner(os.getpid(), " ".join(sys.argv[1:]))

    def _term(signum, frame):
        lib.log(f"worker received signal {signum}: stopping after the current step")
        lib.stop_flag = True
        C.kill_running()
    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, _term)
    try:
        if a.one:
            al, be = [float(x) for x in a.one.split(",")]
            lib.run_one_thread(al, be, a.nproc)
        else:
            lib.start_thread([float(x) for x in a.alphas.split(",")], [float(x) for x in a.betas.split(",")], a.nproc)
        last = 0
        while lib.running and lib.thread is not None and lib.thread.is_alive():
            time.sleep(2)
            if not a.worker and len(lib.log_lines) > last:
                print("\n".join(lib.log_lines[last:]), flush=True)
                last = len(lib.log_lines)
        if not a.worker:
            print("\n".join(lib.log_lines[last:]))
            print(json.dumps({k: v for k, v in lib.status().items() if k not in ("log", "cases", "live", "runner")}, indent=1))
    finally:
        lib.clear_runner()
