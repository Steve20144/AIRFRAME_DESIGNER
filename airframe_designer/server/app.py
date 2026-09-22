"""FastAPI web server: serves the 3D UI, streams sim state over a websocket, exposes control REST endpoints."""
from __future__ import annotations

import asyncio
import json
import time
from collections import deque
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from starlette.concurrency import run_in_threadpool

from ..geometry.airframe import Airframe, PRESETS
from ..geometry.gear import generate_legs
from ..geometry.paths import list_paths, apply_variables
from ..analysis import static as design
from ..analysis.geometric_optimiser import optimise as geometric_optimise, default_groups
from ..px4.link import mavlink, DEBUG_VECT_ID
from ..px4 import param_meta
from ..px4.sitl import instance_is_free
from ..sim.nose_lift import uses_firmware
from ..aero import airfoils
from ..geometry import cad as cadmod
from . import tuning

PROJECT_DIR = Path(__file__).resolve().parents[2]
UI_DIR = PROJECT_DIR / "ui"
AIRFRAME_DIR = PROJECT_DIR / "airframes"
SCENARIO_DIR = PROJECT_DIR / "scenarios"
STUDY_DIR = PROJECT_DIR / "studies"
RESULTS_DIR = PROJECT_DIR / "results"

# PX4 custom mode encoding: main_mode << 16 | sub_mode << 24
PX4_MODES = {
    "manual": (1, 0), "altitude": (2, 0), "position": (3, 0), "acro": (5, 0), "stabilized": (7, 0),
    "takeoff": (4, 2), "hold": (4, 3), "mission": (4, 4), "rtl": (4, 5), "land": (4, 6),
}
PX4_MAIN_MODE_NAMES = {1: "Manual", 2: "Altitude", 3: "Position", 4: "Auto", 5: "Acro", 6: "Offboard", 7: "Stabilized",
                       8: "Rattitude", 9: "Simple", 10: "Termination"}
PX4_SUB_MODE_NAMES = {1: "Ready", 2: "Takeoff", 3: "Hold", 4: "Mission", 5: "RTL", 6: "Land", 8: "Follow", 9: "Precland"}


def mode_name(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    name = PX4_MAIN_MODE_NAMES.get(main, f"mode{main}")
    if main == 4:
        name = PX4_SUB_MODE_NAMES.get(sub, f"Auto{sub}")
    return name


def json_safe(x):
    """Replace non-finite floats (which the JSON encoder refuses) with None, recursively."""
    import math
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: json_safe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [json_safe(v) for v in x]
    return x


class _NoLink:
    """Stand-in while no PX4 link exists so the endpoints degrade gracefully."""
    mode = "none"
    connected = False
    ctl_connected = False
    params: dict = {}
    param_count = 0

    def status(self):
        return {"mode": "none", "address": "", "connected": False, "ctl_connected": False, "armed": False,
                "hil_enabled": False, "custom_mode": 0, "rx_count": 0, "param_count": 0, "params_loaded": 0,
                "actuator_seq": 0, "qgc_proxy": None, "target_system": 0, "ctl_address": ""}

    def __getattr__(self, name):
        def noop(*a, **k):
            return {"ok": False, "error": "PX4 not connected"}
        return noop


class AppState:
    def __init__(self, simulator, conn, args, log_buffer: deque, log):
        self.simulator = simulator
        self.conn = conn                     # ConnectionManager
        self.args = args
        self.log_buffer = log_buffer
        self.log = log
        self.meta: dict[str, dict] = {}
        self.meta_source = ""
        self.export_log: deque = deque(maxlen=500)
        self.opt_job: dict = {"running": False, "progress": 0.0, "message": "", "result": None, "error": None}
        self.batch_jobs: dict[str, dict] = {}
        self.study_job: dict = {"running": False}
        self.scenario_job: dict = {"runner": None}
        # the airframe as it was before a live scenario applied its attitude block, with a fingerprint of what the
        # scenario put on the live simulator: (base Airframe, attitude'd airframe as dict). See base_airframe().
        self.attitude_base: tuple | None = None

    @property
    def link(self):
        return self.conn.link if self.conn.link is not None else _NoLink()


def build_app(state: AppState) -> FastAPI:
    app = FastAPI(title="AIRFRAME_DESIGNER")
    app.mount("/static", StaticFiles(directory=str(UI_DIR)), name="static")
    (AIRFRAME_DIR / "meshes").mkdir(exist_ok=True)
    app.mount("/meshes", StaticFiles(directory=str(AIRFRAME_DIR / "meshes")), name="meshes")   # CAD visuals

    @app.middleware("http")
    async def no_cache(request, call_next):
        # the UI is edited live; browsers otherwise keep stale copies of app.js/scene.js across reloads
        response = await call_next(request)
        if request.url.path.startswith("/static/") or request.url.path == "/":
            response.headers["Cache-Control"] = "no-store, must-revalidate"
            response.headers["Pragma"] = "no-cache"
        return response
    sim = state.simulator

    class _LinkProxy:
        def __getattr__(self, name):
            return getattr(state.link, name)

    link = _LinkProxy()   # always resolves to the current link

    # -------------------------------------------------------------- pages
    @app.get("/")
    async def index():
        return FileResponse(str(UI_DIR / "index.html"))

    # ------------------------------------------------------------- status
    def status_dict() -> dict:
        s = link.status()
        s["mode_name"] = mode_name(s["custom_mode"])
        s["px4_running"] = state.conn.px4_running()
        s["conn_mode"] = state.conn.mode
        s["conn_error"] = state.conn.error
        s["flashing"] = state.conn.firmware_job.running() and state.conn.firmware_job.action == "upload"
        # arm gating: PX4's last arming-check summary must report no system errors and a usable position
        ready, why = False, "waiting for PX4's arming check report"
        for x in reversed(list(getattr(state.link, "recent_events", []) or [])):
            if x.get("name") == "commander_arming_check_summary":
                d = dict(zip(x.get("arg_names", []), x.get("args", [])))
                # PX4 lists the modes it would arm in; the error mask also carries the always-failing
                # offboard/mission checks ("system"), so it is not usable as a gate on its own.
                can = str(d.get("can_arm", ""))
                mode_now = s.get("mode_name", "").lower()
                aliases = {"hold": "loiter", "stabilized": "stab", "position": "posctl", "altitude": "altctl"}
                ready = ("takeoff" in can) or ("loiter" in can) or (aliases.get(mode_now, mode_now) in can.split("|"))
                why = "" if ready else "PX4 will not arm yet (estimator or health checks); see the Flight tab"
                break
        s["resetting"] = max(0.0, state.conn._reset_busy_until - time.time())
        s["arm_ready"] = bool(s["ctl_connected"]) and (ready or bool(s["armed"])) and s["resetting"] <= 0
        s["arm_block_reason"] = why
        s["px4_ports"] = [p["device"] for p in state.conn.list_ports_cached() if p["likely_px4"]]
        # a RadioMaster radio that shows up as a *serial* port was powered on in VCP/config mode (M + Power);
        # in that mode it is not a joystick
        s["radio_vcp_ports"] = [p["device"] for p in state.conn.list_ports_cached() if "radiomaster" in (p["device"] + p["description"]).lower()]
        s["meta_loaded"] = len(state.meta)
        s["meta_source"] = state.meta_source
        s["home"] = {"lat": sim.sensors.home.lat, "lon": sim.sensors.home.lon, "alt": sim.sensors.home.alt}
        s["speed"] = sim.speed
        s["sensor_rate"] = sim.sensor_rate
        s["lockstep"] = sim.lockstep
        s["noise"] = sim.sensors.noise.enabled
        s["paused"] = sim.paused
        s["physics"] = sim.physics
        return s

    @app.get("/api/status")
    async def get_status():
        return status_dict()

    @app.get("/api/log")
    async def get_log(since: float = 0.0):
        return [e for e in state.log_buffer if e[0] > since]

    # ----------------------------------------------------------- airframe
    AUTOSAVE = AIRFRAME_DIR / "_autosave.json"
    _autosave_t = [0.0]

    def autosave(af: Airframe) -> None:
        """Every accepted edit is written to airframes/_autosave.json (at most twice a second), so a design
        survives a closed terminal or a crash even if it was never saved by name."""
        now = time.time()
        if now - _autosave_t[0] < 0.5:
            return
        _autosave_t[0] = now
        try:
            AIRFRAME_DIR.mkdir(exist_ok=True)
            af.save(AUTOSAVE)
        except Exception as e:
            state.log(f"[ui] autosave failed: {e}")

    @app.get("/api/airframe")
    async def get_airframe():
        return sim.airframe.to_dict()

    @app.post("/api/airframe")
    async def set_airframe(body: dict):
        try:
            af = Airframe.from_dict(body.get("airframe", body))
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"invalid airframe ({type(e).__name__}: {e}); an empty number field?"}, status_code=400)
        keep = bool(body.get("keep_state", True))
        af.resolve_mass()
        try:
            await run_in_threadpool(airfoils.ensure_polars, af, state.log)   # polar wings: section tables ready before the physics
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"airfoil polar: {e}"}, status_code=400)
        sim.set_airframe(af, keep_state=keep)
        autosave(af)
        hc = af.hover_check()
        return json_safe({"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc})

    @app.get("/api/airframe/hover_check")
    async def hover_check():
        return json_safe(sim.airframe.hover_check())

    @app.get("/api/airframes")
    async def list_airframes():
        files = sorted(p.name for p in AIRFRAME_DIR.glob("*.json"))
        return {"presets": list(PRESETS), "files": files}

    @app.post("/api/airframe/load")
    async def load_airframe(body: dict):
        name = body.get("name", "")
        if name in PRESETS:
            af = PRESETS[name]()
        else:
            p = AIRFRAME_DIR / name
            if not p.is_file():
                return JSONResponse({"ok": False, "error": f"not found: {name}"}, status_code=404)
            af = Airframe.load(p)
        try:
            await run_in_threadpool(airfoils.ensure_polars, af, state.log)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"airfoil polar: {e}"}, status_code=400)
        sim.set_airframe(af, keep_state=False)
        hc = af.hover_check()
        return json_safe({"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc})

    def base_airframe():
        """The airframe headless work and saves start from: the live one, unless the live one is still exactly what a
        scenario's attitude block made of it (re-legged, parked and hover pitch moved), in which case the airframe from
        before that scenario. A scenario owns its stance for its own flight only; a later edit by the user (the
        fingerprint no longer matches) makes the live airframe the base again."""
        ab = state.attitude_base
        if ab is None:
            return sim.airframe
        base, fp = ab
        if sim.airframe.to_dict() == fp:
            return base
        state.attitude_base = None
        return sim.airframe

    @app.post("/api/airframe/save")
    async def save_airframe(body: dict):
        name = body.get("name", "").strip()
        if not name:
            return JSONResponse({"ok": False, "error": "name required"}, status_code=400)
        if not name.endswith(".json"):
            name += ".json"
        name = Path(name).name
        AIRFRAME_DIR.mkdir(exist_ok=True)
        af = base_airframe()
        if af is not sim.airframe:
            state.log(f"[airframe] saved {name} at its own stance ({af.landed_pitch_deg:g}/{af.hover_pitch_deg:g} deg), "
                      f"not the live scenario's ({sim.airframe.landed_pitch_deg:g}/{sim.airframe.hover_pitch_deg:g})")
        af.save(AIRFRAME_DIR / name)
        return {"ok": True, "path": str(AIRFRAME_DIR / name)}

    # ------------------------------------------------------------------ CAD (STEP bodies -> masses, CG)
    CAD_DIR = AIRFRAME_DIR / "cad"

    def cad_file_path(model) -> Path:
        p = Path(model.file)
        return p if p.is_absolute() else PROJECT_DIR / p

    @app.post("/api/cad/import")
    async def cad_import(request: Request, filename: str = "model.step"):
        """Upload a STEP file (raw bytes): it is copied to airframes/cad/, every solid measured and meshed, and the
        bodies attached to the live airframe (masses/offsets of bodies with the same id are kept on re-import)."""
        data = await request.body()
        if not data:
            return JSONResponse({"ok": False, "error": "empty upload"}, status_code=400)
        CAD_DIR.mkdir(parents=True, exist_ok=True)
        stem = cadmod._safe_stem(filename)
        suffix = ".stp" if filename.lower().endswith(".stp") else ".step"
        dst = CAD_DIR / (stem + suffix)
        dst.write_bytes(data)
        try:
            imported = await run_in_threadpool(cadmod.import_step, dst, state.log)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"STEP import failed: {e}"}, status_code=400)
        af = sim.airframe.copy()
        af.cad = cadmod.model_from_import(imported, str(dst.relative_to(PROJECT_DIR)), af.cad)
        af.resolve_mass()
        sim.set_airframe(af, keep_state=True)
        autosave(af)
        return {"ok": True, "airframe": json_safe(af.to_dict()), "bodies": len(af.cad.bodies),
                "mesh": cadmod.mesh_payload(af.cad, imported)}

    @app.get("/api/cad/mesh")
    async def cad_mesh():
        """Meshes of the live airframe's CAD bodies in the structural frame (axes/origin/scale applied, offsets not)."""
        m = sim.airframe.cad
        if not m:
            return {"ok": True, "file": None, "bodies": []}
        path = cad_file_path(m)
        if not path.exists():
            return JSONResponse({"ok": False, "error": f"CAD file missing: {m.file}"}, status_code=404)
        try:
            imported = await run_in_threadpool(cadmod.import_step, path, state.log)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"STEP import failed: {e}"}, status_code=400)
        return cadmod.mesh_payload(m, imported)

    @app.get("/api/cad/totals")
    async def cad_totals():
        m = sim.airframe.cad
        return {"ok": True, "totals": m.totals() if m else None}

    @app.post("/api/airframe/estimate_inertia")
    async def estimate_inertia():
        af = sim.airframe
        af.mass.inertia = af.estimate_inertia()
        sim.set_airframe(af)
        return {"ok": True, "inertia": af.mass.inertia}

    @app.post("/api/airframe/legs/generate")
    async def legs_generate(body: dict):
        """Four legs from a height below the CG, spreads and the landed pitch (replaces the airframe's legs)."""
        af = sim.airframe
        kw = {}
        if body.get("stiffness") is not None:
            kw["stiffness"] = float(body["stiffness"]); kw["damping"] = float(body.get("damping", 150.0))
        legs = generate_legs(float(body.get("height", 0.2)), float(body.get("spread_x", 0.2)), float(body.get("spread_y", body.get("spread_x", 0.2))),
                             float(body.get("landed_pitch_deg", af.landed_pitch_deg)), float(body.get("attach_z", 0.0)), cg=af.mass.cg,
                             mass=af.mass.mass, **kw)
        return {"ok": True, "legs": [l.to_dict() for l in legs]}

    @app.post("/api/airframe/legs/auto")
    async def legs_auto(body: dict | None = None):
        """Size every leg's spring/damper from the mass (2 cm static sink, damping ratio 0.8 by default)."""
        body = body or {}
        af = sim.airframe
        af.auto_leg_constants(float(body.get("compression_m", 0.02)), float(body.get("zeta", 0.8)))
        sim.set_airframe(af, keep_state=True)
        autosave(af)
        return {"ok": True, "legs": [l.to_dict() for l in af.legs], "static": af.leg_static()}

    @app.get("/api/airfoils")
    async def list_airfoils():
        return airfoils.list_airfoils()

    @app.get("/api/airfoil/coords")
    async def airfoil_coords(name: str):
        """Section coordinates (chord-normalised, Selig order: upper TE -> LE -> lower TE) for drawing."""
        try:
            pts = await run_in_threadpool(airfoils.load_coordinates, name)
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=404)
        return {"ok": True, "name": airfoils.normalise_name(name), "coords": [[round(float(x), 5), round(float(y), 5)] for x, y in pts]}

    @app.post("/api/airfoil/polar")
    async def airfoil_polar(body: dict):
        """Build (or load) the polar table of an airfoil by name (NACA 4/5-digit, a local airfoils/<name>.dat, or a
        UIUC database name) and return its headline numbers. {"name": "naca23006", "force": false, "re": 5e5}"""
        name = str(body.get("name", "")).strip()
        if not name:
            return JSONResponse({"ok": False, "error": "name required"}, status_code=400)
        try:
            table = await run_in_threadpool(airfoils.build_polar, name, None, float(body.get("ncrit", 9.0)), str(body.get("source", "auto")), bool(body.get("force", False)))
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)
        Re = float(body.get("re", 5e5))
        return {"ok": True, "name": table["name"], "source": table["source"], "built_s": table.get("built_s"), "note": table.get("note"),
                "re_list": table["re"], "summary": airfoils.polar_summary(table, Re), "valid": table["valid"],
                "curve": {"alpha": table["alpha"][::4], "cl": table["cl"][min(3, len(table["re"]) - 1)][::4], "cd": table["cd"][min(3, len(table["re"]) - 1)][::4]}}

    @app.get("/api/airframe/paths")
    async def airframe_paths():
        return {"paths": list_paths(sim.airframe)}

    # ---------------------------------------------------------------- design
    def design_speed(body: dict | None) -> float:
        kmh = (body or {}).get("speed_kmh")
        if kmh is None:
            kmh = sim.airframe.design.get("cruise_speed_kmh", 50.0)
        return float(kmh) / 3.6

    def design_notes(speed: float) -> list[str]:
        """Things PX4 must allow for the cruise case: velocity limits and tilt."""
        notes = []
        v = link.params.get("MPC_XY_VEL_MAX", {}).get("value")
        if v is not None and speed > float(v) + 1e-6:
            notes.append(f"MPC_XY_VEL_MAX is {float(v):g} m/s, cruise needs {speed:.1f} m/s")
        c = link.params.get("MPC_XY_CRUISE", {}).get("value")
        if c is not None and speed > float(c) + 1e-6:
            notes.append(f"MPC_XY_CRUISE is {float(c):g} m/s, missions fly at that speed")
        return notes

    @app.post("/api/design/analysis")
    async def design_analysis(body: dict | None = None):
        body = body or {}
        if "airframe" in body:
            try:
                af = Airframe.from_dict(body["airframe"])
            except Exception as e:
                return JSONResponse({"ok": False, "error": f"invalid airframe: {e}"}, status_code=400)
        else:
            af = sim.airframe
        speed = design_speed(body)
        tilt = link.params.get("MPC_TILTMAX_AIR", {}).get("value")
        tilt = float(tilt) if tilt is not None else 45.0
        r = await run_in_threadpool(design.analyse, af, speed, tilt)
        r["notes"] = design_notes(speed)
        r["tilt_limit_deg"] = tilt
        r["groups"] = default_groups(af)
        return json_safe(r)

    @app.post("/api/design/optimize")
    async def design_optimize(body: dict):
        job = state.opt_job
        if job["running"]:
            return JSONResponse({"ok": False, "error": "an optimisation is already running"}, status_code=409)
        try:
            af = Airframe.from_dict(body["airframe"]) if "airframe" in body else sim.airframe
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"invalid airframe: {e}"}, status_code=400)
        spec = dict(body.get("spec") or {})
        spec.setdefault("speed_kmh", af.design.get("cruise_speed_kmh", 50.0))
        tilt = link.params.get("MPC_TILTMAX_AIR", {}).get("value")
        spec.setdefault("tilt_limit_deg", float(tilt) if tilt is not None else 45.0)
        job.update(running=True, progress=0.0, message="starting", result=None, error=None)

        def progress(frac, msg):
            job["progress"], job["message"] = float(frac), str(msg)

        def run():
            try:
                job["result"] = geometric_optimise(af, spec, progress)
            except Exception as e:
                job["error"] = f"{type(e).__name__}: {e}"
                state.log(f"[design] optimisation failed: {e}")
            finally:
                job["running"] = False

        import threading
        threading.Thread(target=run, name="design-optimise", daemon=True).start()
        return {"ok": True}

    @app.get("/api/design/optimize")
    async def design_optimize_status():
        j = state.opt_job
        return {"running": j["running"], "progress": j["progress"], "message": j["message"],
                "result": j["result"], "error": j["error"]}

    # ------------------------------------------------------------ connection
    @app.get("/api/connection")
    async def get_connection():
        st = state.conn.status()
        st["checklist"] = state.conn.checklist(export_params() if state.conn.mode == "hitl" else None)
        return st

    @app.post("/api/connection/connect")
    async def connect(body: dict):
        mode = body.get("mode", "sitl")
        sim.paused = False          # a paused loop sends PX4 nothing; connecting while paused can only fail
        if mode == "hitl":
            r = await run_in_threadpool(state.conn.connect_hitl, body.get("serial"), body.get("baud"))
        else:
            r = await run_in_threadpool(state.conn.connect_sitl, body.get("launch"))
        return r

    @app.post("/api/connection/disconnect")
    async def disconnect():
        return await run_in_threadpool(state.conn.disconnect)

    @app.post("/api/px4/reset_all")
    async def px4_reset_all():
        sim.paused = False
        r = await run_in_threadpool(state.conn.reset_all)
        state.log("[px4] reset: " + ", ".join(r.get("steps", [])))
        return r

    @app.post("/api/px4/recover")
    async def px4_recover():
        r = await run_in_threadpool(state.conn.recover)
        state.log("[px4] recover: " + ", ".join(r.get("steps", [])))
        return r

    @app.post("/api/connection/restart_estimator")
    async def restart_estimator():
        return await run_in_threadpool(state.conn.restart_estimator)

    @app.post("/api/connection/enable_hitl")
    async def enable_hitl():
        return await run_in_threadpool(state.conn.enable_hitl)

    # ---- USB passthrough (WSL only): the board is a Windows COM port until usbipd hands it over
    @app.get("/api/usb")
    async def usb_status():
        from ..px4.usbip import status
        return await run_in_threadpool(status)

    @app.post("/api/usb/attach")
    async def usb_attach(body: dict | None = None):
        from ..px4.usbip import attach
        r = await run_in_threadpool(attach, (body or {}).get("busid"))
        state.log("[usb] " + str(r.get("message", "")))
        if r.get("hint"):
            state.log("[usb] " + r["hint"])
        if r.get("ok"):
            # The tty takes a moment to enumerate after usbipd hands the device over, and the UI connects to it
            # straight after this returns. Wait for it here rather than let that connect fail on an empty scan.
            def wait_for_port(deadline: float = 6.0) -> None:
                t0 = time.time()
                while time.time() - t0 < deadline:
                    if any(pt.get("likely_px4") for pt in state.conn.list_ports_cached(0.0)):
                        return
                    time.sleep(0.25)
            await run_in_threadpool(wait_for_port)
        return r

    @app.post("/api/usb/detach")
    async def usb_detach(body: dict | None = None):
        from ..px4.usbip import detach
        r = await run_in_threadpool(detach, (body or {}).get("busid"))
        state.log("[usb] " + str(r.get("message", "")))
        return r

    @app.post("/api/firmware/build")
    async def firmware_build(body: dict | None = None):
        return await run_in_threadpool(state.conn.build_firmware, (body or {}).get("target"))

    @app.post("/api/firmware/upload")
    async def firmware_upload(body: dict | None = None):
        return await run_in_threadpool(state.conn.upload_firmware, (body or {}).get("target"))

    @app.post("/api/px4/shell")
    async def px4_shell(body: dict):
        cmd = str(body.get("command", "")).strip()
        if not cmd:
            return JSONResponse({"ok": False, "error": "command required"}, status_code=400)
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 not connected"}, status_code=409)
        out = await run_in_threadpool(link.shell, cmd, float(body.get("timeout", 3.0)))
        return {"ok": True, "output": out}

    @app.get("/api/rc")
    async def get_rc():
        rc = dict(getattr(state.link, "rc", {}) or {})
        rc = rc if rc and time.time() - rc.get("t", 0) < 3.0 else {}
        # PX4 channel mapping (1-based channel numbers, 0 = unassigned)
        names = {"RC_MAP_ROLL": "Roll", "RC_MAP_PITCH": "Pitch", "RC_MAP_THROTTLE": "Throttle", "RC_MAP_YAW": "Yaw",
                 "RC_MAP_FLTMODE": "Flight mode", "RC_MAP_ARM_SW": "Arm", "RC_MAP_KILL_SW": "Kill", "RC_MAP_RETURN_SW": "Return",
                 "RC_MAP_LOITER_SW": "Loiter", "RC_MAP_OFFB_SW": "Offboard", "RC_MAP_GEAR_SW": "Gear", "RC_MAP_FLAPS": "Flaps",
                 "RC_MAP_AUX1": "Aux 1", "RC_MAP_AUX2": "Aux 2", "RC_MAP_AUX3": "Aux 3", "RC_MAP_AUX4": "Aux 4",
                 "RC_MAP_AUX5": "Aux 5", "RC_MAP_AUX6": "Aux 6", "RC_MAP_PARAM1": "Param 1", "RC_MAP_PARAM2": "Param 2",
                 "RC_MAP_PARAM3": "Param 3", "RC_MAP_TRANS_SW": "Transition", "RC_MAP_ENG_MOT": "Engine/motor",
                 "RC_MAP_PAY_SW": "Payload", "RC_MAP_FAILSAFE": "Failsafe"}
        mapping: dict[int, list[str]] = {}
        params = getattr(state.link, "params", {}) or {}
        for k, label in names.items():
            v = params.get(k, {}).get("value")
            if isinstance(v, (int, float)) and int(v) > 0:
                mapping.setdefault(int(v), []).append(label)
        rc["mapping"] = {str(k): v for k, v in mapping.items()}
        rc["rc_in_mode"] = params.get("COM_RC_IN_MODE", {}).get("value")
        return rc

    @app.get("/api/events")
    async def get_events():
        return {"source": state.conn.event_decoder.source if state.conn.event_decoder else "",
                "events": list(link.recent_events) if hasattr(state.link, "recent_events") else []}

    @app.post("/api/events/meta/fetch")
    async def fetch_events_meta():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        local, msg = await run_in_threadpool(param_meta.fetch_extra, link, "all_events.json.xz", state.log)
        if local is None:
            return JSONResponse({"ok": False, "error": msg}, status_code=500)
        from .events import _read_json
        n = state.conn.event_decoder.load(_read_json(local), str(local))
        return {"ok": True, "count": n}

    @app.get("/api/firmware")
    async def firmware_status():
        b = state.conn.detected_board()
        return {"job": state.conn.firmware_job.status(), "board": b, "toolchain": state.conn.toolchain_present(),
                "built": state.conn.firmware_file(b["target"])}

    # ---- Controller tab: which transmitter control drives which channel (learned by moving it), and what each
    # control should do. That belongs to the radio, not the airframe, so it is kept per user, not per airframe.
    controller_file = Path.home() / ".airframe_designer" / "controller.json"

    @app.get("/api/controller")
    async def get_controller():
        try:
            return json.loads(controller_file.read_text())
        except (OSError, ValueError):
            return {}

    @app.post("/api/controller")
    async def save_controller(body: dict):
        controller_file.parent.mkdir(parents=True, exist_ok=True)
        controller_file.write_text(json.dumps(body, indent=2))
        return {"ok": True}

    from ..px4.killtest import KillTest
    kill_test = KillTest(lambda: state.link, state.log)

    @app.get("/api/killtest")
    async def killtest_status():
        return kill_test.status()

    @app.post("/api/killtest")
    async def killtest_control(body: dict):
        return kill_test.start() if body.get("action") == "start" else kill_test.stop()

    # ---- Flash tab: build and flash the firmware images (px4/firmware_images.py) with a live log
    @app.get("/api/flash")
    async def flash_status(target: str | None = None):
        from ..px4.usbip import status as usb_status_, auto_attach_running
        imgs = await run_in_threadpool(state.conn.flash_images, target)
        usb = await run_in_threadpool(usb_status_)
        usb["auto_attach"] = await run_in_threadpool(auto_attach_running) if usb.get("available") else False
        link_ = state.link
        return {**imgs, "board": state.conn.detected_board(), "usb": usb, "job": state.conn.firmware_job.status(),
                "toolchain": state.conn.toolchain_present(),
                "connection": {"mode": state.conn.mode, "serial": state.conn.serial,
                               "connected": bool(link_ is not None and getattr(link_, "ctl_connected", False)),
                               "firmware": getattr(link_, "firmware", {}) if link_ is not None else {}}}

    @app.get("/api/flash/log")
    async def flash_log(since: int = 0):
        return {**state.conn.firmware_job.lines_since(int(since)), "job": state.conn.firmware_job.status()}

    @app.post("/api/flash/build")
    async def flash_build(body: dict):
        return await run_in_threadpool(state.conn.flash_build, str(body.get("image", "")), body.get("target"))

    @app.post("/api/flash/upload")
    async def flash_upload(body: dict):
        return await run_in_threadpool(state.conn.flash_upload, str(body.get("image", "")), body.get("target"))

    @app.post("/api/flash/cancel")
    async def flash_cancel():
        job = state.conn.firmware_job
        if job.running() and job.action == "upload" and job.progress.get("phase") in ("erase", "program", "verify"):
            return JSONResponse({"ok": False, "error": "the board is being written: stopping now would leave it without "
                                 "firmware; let it finish"}, status_code=409)
        return {"ok": job.cancel()}

    @app.post("/api/flash/verify")
    async def flash_verify():
        return await run_in_threadpool(state.conn.flash_verify)

    # ------------------------------------------------------------ PX4 export
    def export_params() -> dict[str, float | int]:
        hitl = link.mode == "hitl"
        return sim.airframe.px4_params(hitl=True) if hitl else sim.airframe.px4_params_sitl()

    @app.get("/api/px4/export")
    async def get_export():
        params = export_params()
        current = {k: link.params.get(k, {}).get("value") for k in params}
        ov = sim.airframe.px4_overrides or {}
        return json_safe({"params": params, "current": current, "problems": sim.airframe.validate() + sim.airframe.hover_check()["problems"],
                "file": sim.airframe.px4_params_file(hitl=link.mode == "hitl"),
                "overrides": ov, "geometry_keys": [k for k in params if k not in ov]})

    @app.get("/api/px4/export.params")
    async def get_export_file():
        return PlainTextResponse(sim.airframe.px4_params_file(hitl=link.mode == "hitl"),
                                 headers={"Content-Disposition": f'attachment; filename="{sim.airframe.name}.params"'})

    @app.post("/api/px4/push")
    async def push_params(body: dict | None = None):
        body = body or {}
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        if link.armed:
            return JSONResponse({"ok": False, "error": "vehicle is armed; disarm before updating PX4"}, status_code=409)
        params = export_params()
        only = body.get("only")
        if only:
            params = {k: v for k, v in params.items() if k in only}
        # skip output-function params the firmware does not have (e.g. HIL_ACT on SITL)
        if link.params:
            missing = [k for k in params if k not in link.params]
            params = {k: v for k, v in params.items() if k in link.params}
        else:
            missing = []
        state.export_log.clear()
        rot_before = link.params.get("SENS_BOARD_Y_OFF", {}).get("value")

        def progress(name, res):
            state.export_log.append({"t": time.time(), **res})

        results = await run_in_threadpool(link.set_params, params, progress)
        ok = all(r["ok"] for r in results)
        if body.get("save", True) and ok:
            link.preflight_storage(True)
        rot_after = params.get("SENS_BOARD_Y_OFF")
        if ok and rot_after is not None and rot_before is not None and abs(float(rot_after) - float(rot_before)) > 1e-3:
            # the IMU frame just changed under the running estimator: rest the sim at the new hover attitude and
            # restart EKF2 so it aligns from clean data
            state.log(f"[export] board rotation changed ({rot_before} -> {rot_after} deg): resetting sim, restarting estimator")
            sim.reset()
            await run_in_threadpool(state.conn.restart_estimator)     # HITL: reboots the board
        failed = [r for r in results if not r["ok"]]
        state.log(f"[export] pushed {len(results) - len(failed)}/{len(results)} params to PX4"
                  + (f", failed: {[r['name'] for r in failed]}" if failed else ""))
        return {"ok": ok, "results": results, "missing": missing}

    # ---------------------------------------------------------- parameters
    @app.get("/api/params")
    async def get_params():
        return {"count": link.param_count, "params": link.params}

    @app.post("/api/params/refresh")
    async def refresh_params():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        params = await run_in_threadpool(link.fetch_all_params)
        return {"ok": True, "count": link.param_count, "params": params}

    @app.post("/api/params/set")
    async def set_param(body: dict):
        name = body.get("name")
        value = body.get("value")
        if name is None or value is None:
            return JSONResponse({"ok": False, "error": "name and value required"}, status_code=400)
        res = await run_in_threadpool(link.set_param, name, value)
        if res.get("ok"):
            # remember it with the airframe so Save keeps it and Update PX4 re-applies it
            sim.airframe.px4_overrides[name] = res["value"]
        return res

    @app.post("/api/airframe/override")
    async def set_override(body: dict):
        """Record an edited parameter without touching the vehicle (used when not connected)."""
        name, value = body.get("name"), body.get("value")
        if not name or value is None:
            return JSONResponse({"ok": False, "error": "name and value required"}, status_code=400)
        sim.airframe.px4_overrides[name] = value
        return {"ok": True, "overrides": sim.airframe.px4_overrides}

    @app.post("/api/airframe/override_remove")
    async def remove_override(body: dict):
        sim.airframe.px4_overrides.pop(body.get("name", ""), None)
        return {"ok": True, "overrides": sim.airframe.px4_overrides}

    @app.post("/api/params/save")
    async def save_params():
        link.preflight_storage(True)
        return {"ok": True}

    @app.get("/api/params/meta")
    async def get_meta():
        return {"source": state.meta_source, "meta": state.meta}

    @app.post("/api/params/meta/fetch")
    async def fetch_meta():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        meta, src = await run_in_threadpool(param_meta.fetch_from_vehicle, link, state.log)
        if meta:
            state.meta, state.meta_source = meta, src
            return {"ok": True, "count": len(meta), "source": src}
        return JSONResponse({"ok": False, "error": src}, status_code=500)

    # ------------------------------------------------------------ vehicle
    @app.post("/api/px4/command")
    async def px4_command(body: dict):
        cmd = body.get("command", "")
        if cmd in ("arm", "takeoff") and sim.paused:
            return JSONResponse({"ok": False, "error": "the simulation is paused; resume it first (Pause button)"}, status_code=409)
        if cmd == "arm":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0 if body.get("force") else 0.0)
        elif cmd == "disarm":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0 if body.get("force") else 0.0)
        elif cmd == "kill":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        elif cmd == "mode":
            main, sub = PX4_MODES.get(body.get("mode", ""), (None, None))
            if main is None:
                return JSONResponse({"ok": False, "error": "unknown mode"}, status_code=400)
            link.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                                   float(main), float(sub))
        elif cmd == "takeoff":
            main, sub = PX4_MODES["takeoff"]
            link.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                                   float(main), float(sub))
        elif cmd == "reboot":
            link.reboot()
        elif cmd == "save_params":
            link.preflight_storage(True)
        else:
            return JSONResponse({"ok": False, "error": "unknown command"}, status_code=400)
        return {"ok": True}

    # ---------------------------------------------------------------- sim
    @app.post("/api/sim/reset")
    async def sim_reset(body: dict | None = None):
        body = body or {}
        sim.reset(yaw=float(body.get("yaw", 0.0)))
        return {"ok": True}

    @app.post("/api/sim/pause")
    async def sim_pause(body: dict):
        sim.paused = bool(body.get("paused", not sim.paused))
        return {"ok": True, "paused": sim.paused}

    @app.post("/api/sim/speed")
    async def sim_speed(body: dict):
        sim.speed = max(0.0, float(body.get("speed", 1.0)))
        return {"ok": True, "speed": sim.speed}

    @app.post("/api/sim/motor_override")
    async def motor_override(body: dict):
        v = body.get("values")
        sim.motor_override = None if v is None else [float(x) for x in v]
        return {"ok": True}

    @app.post("/api/sim/physics")
    async def sim_physics(body: dict):
        """Switch the live physics engine: {"physics": "python" | "jsbsim"}. The vehicle restarts on the ground."""
        name = str(body.get("physics", "python")).lower()
        if name not in sim.BACKENDS:
            return JSONResponse({"ok": False, "error": f"unknown physics '{name}'"}, status_code=400)
        if link.armed:
            return JSONResponse({"ok": False, "error": "disarm first"}, status_code=409)
        try:
            await run_in_threadpool(sim.set_physics, name)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"{type(e).__name__}: {e}"}, status_code=500)
        return {"ok": True, "physics": sim.physics}

    @app.post("/api/sim/nose_lift")
    async def sim_nose_lift(body: dict):
        """Start ({"motors": [8, 9], "target_pitch_deg": 25, "rate_deg_s": 8}) or stop ({"stop": true}) the nose-lift
        ground sequence. The snapshot's "nose_lift" field reports its state; arm PX4 once it is "holding"."""
        if body.get("stop"):
            sim.stop_nose_lift()
            return {"ok": True}
        if uses_firmware(getattr(sim.airframe, "design", None)):
            return JSONResponse({"ok": False, "error": "the flight controller runs this airframe's nose lift: arm, then "
                                 "flip the RC switch"}, status_code=409)
        if link.armed:
            return JSONResponse({"ok": False, "error": "disarm first: the nose lift runs before arming"}, status_code=409)
        if not sim.sim.on_ground:
            return JSONResponse({"ok": False, "error": "the vehicle is not on the ground"}, status_code=409)
        motors = [int(m) for m in body.get("motors") or []]
        if not motors:
            return JSONResponse({"ok": False, "error": "choose the motors that lift the nose"}, status_code=400)
        kw = {k: float(body[k]) for k in ("rate_deg_s", "kp", "ki", "kd", "max_cmd", "tolerance_deg", "hold_s", "fade_s", "k_ang", "kq", "kqi", "assist_cmd") if k in body}
        if body.get("assist_motors"):
            kw["assist_motors"] = [int(m) for m in body["assist_motors"]]
        nl = sim.start_nose_lift(motors, float(body.get("target_pitch_deg", sim.airframe.hover_pitch_deg)), **kw)
        return {"ok": True, "status": nl.status()}

    @app.post("/api/sim/nose_lower")
    async def sim_nose_lower(body: dict):
        """Start the landing ground sequence ({"motors": [8, 9], "target_pitch_deg": 8, "rate_deg_s": 3,
        "wait_touchdown": true}) or stop it ({"stop": true}). With wait_touchdown it arms in the air and takes over
        at touchdown (rear motors cut, front motors lower the nose); with design.nose_lower.enabled the simulator
        arms it by itself on every flight."""
        if body.get("stop"):
            sim.stop_nose_lift()
            return {"ok": True}
        motors = [int(m) for m in body.get("motors") or []]
        if not motors:
            return JSONResponse({"ok": False, "error": "choose the motors that hold the nose"}, status_code=400)
        kw = {k: float(body[k]) for k in ("rate_deg_s", "min_airborne_alt", "fade_s", "tolerance_deg", "timeout_s", "wait_timeout_s", "k_ang", "kq", "kqi", "max_cmd", "takeover_boost", "rear_fade_s") if k in body}
        kw["wait_touchdown"] = bool(body.get("wait_touchdown", True))
        if not kw["wait_touchdown"] and not sim.sim.on_ground:
            return JSONResponse({"ok": False, "error": "the vehicle is not on the ground; use wait_touchdown"}, status_code=409)
        nl = sim.start_nose_lower(motors, float(body.get("target_pitch_deg", sim.airframe.landed_pitch_deg)), **kw)
        return {"ok": True, "status": nl.status()}

    def rc_switch_loop() -> None:
        """A transmitter switch that runs the nose-lift ground sequence before arming. The switch is
        design.nose_lift.rc_channel (1-based RC_CHANNELS index, 0 = off); the switch is "on" above rc_threshold
        (1500), or below it with rc_active_low. An off-to-on edge while PX4 is disarmed and the vehicle is on its
        legs starts the lift with the card's settings, an on-to-off edge while still disarmed stops it. When the
        pilot arms with the switch still on and the nose holding, Takeoff mode is requested (rc_takeoff, default
        true): that makes the switch a complete takeoff sequence. Edge-triggered, so a switch that is already on
        at start does nothing."""
        last: bool | None = None
        land_last: bool | None = None
        takeoff_sent = False
        while True:
            time.sleep(0.1)
            try:
                d = (getattr(sim.airframe, "design", None) or {}).get("nose_lift") or {}
                ch = int(d.get("rc_channel") or 0)
                if uses_firmware(getattr(sim.airframe, "design", None)):
                    # executor "firmware": the flight controller reads the switch itself (NL_RC_CH); keep its
                    # state streaming (DEBUG_VECT "NLIFT"), which PX4 does not send on USB by default
                    fw = getattr(state.link, "nose_lift_fw", {}) or {}
                    stale = time.time() - fw.get("t", 0) > 3.0
                    if getattr(state.link, "ctl_connected", False) and stale and \
                            time.time() - getattr(rc_switch_loop, "_nl_req", 0.0) > 5.0:
                        rc_switch_loop._nl_req = time.time()
                        state.link.request_message(DEBUG_VECT_ID, 10.0)
                    last = None; land_last = None
                    continue
                if ch <= 0:
                    last = None; land_last = None
                    continue
                rc = getattr(state.link, "rc", {}) or {}
                vals = rc.get("channels") or []
                if not rc or time.time() - rc.get("t", 0) > 1.0 or ch > len(vals):
                    continue
                v = float(vals[ch - 1])
                thr = float(d.get("rc_threshold", 1500))
                high = v < thr if d.get("rc_active_low") else v > thr
                # the other end of a three-position switch is the landing rotation (nose down onto the front leg):
                # mirror of the takeoff threshold about 1500 unless rc_land_threshold says otherwise
                lthr = float(d.get("rc_land_threshold", 3000 - thr))
                land = v > lthr if d.get("rc_active_low") else v < lthr
                if land_last is None:
                    land_last = land
                elif land and not land_last:
                    dl = (getattr(sim.airframe, "design", None) or {}).get("nose_lower") or {}
                    motors = [int(m) for m in dl.get("motors") or d.get("motors") or []]
                    if sim.nose_lift is not None and getattr(sim.nose_lift, "kind", "lift") == "lift" and not link.armed:
                        sim.stop_nose_lift()
                    if link.armed or not sim.sim.on_ground:
                        state.log(f"[rc] channel {ch} land position while flying: the nose lowers by itself on touchdown"
                                  if dl.get("enabled", True) else f"[rc] channel {ch} land position while flying: ignored")
                    elif not motors:
                        state.log(f"[rc] channel {ch} land position but no nose motors chosen")
                    else:
                        sim.start_nose_lower(motors, float(dl.get("target_pitch_deg", sim.airframe.landed_pitch_deg)),
                                             rate_deg_s=float(dl.get("rate_deg_s", 3)), fade_s=float(dl.get("fade_s", 4)),
                                             wait_touchdown=False)
                        state.log(f"[rc] channel {ch} land position: lowering the nose to {sim.airframe.landed_pitch_deg:g} deg")
                land_last = land
                if last is None:
                    last = high
                    continue
                nl = sim.nose_lift
                if high and last and nl is not None and getattr(nl, "kind", "lift") == "lift" and link.armed                         and nl.state in ("holding", "handover") and d.get("rc_takeoff", True) and not takeoff_sent:
                    # the pilot armed while the switch is still on: this is the takeoff sequence, climb to MIS_TAKEOFF_ALT
                    link.set_mode("takeoff")
                    takeoff_sent = True
                    state.log(f"[rc] channel {ch} on and PX4 armed with the nose holding: Takeoff requested")
                if not high:
                    takeoff_sent = False
                if high and not last:
                    motors = [int(m) for m in d.get("motors") or []]
                    if link.armed or not sim.sim.on_ground or sim.nose_lift is not None or not motors:
                        state.log(f"[rc] channel {ch} high but the nose lift cannot start "
                                  f"({'armed' if link.armed else 'airborne' if not sim.sim.on_ground else 'already running' if sim.nose_lift is not None else 'no motors chosen'})")
                    else:
                        kw: dict = {"rate_deg_s": float(d.get("rate_deg_s", 8))}
                        if float(d.get("assist_cmd", 0) or 0) > 0:
                            kw["assist_motors"] = [i for i in range(len(sim.airframe.rotors)) if i not in motors]
                            kw["assist_cmd"] = float(d["assist_cmd"])
                        sim.start_nose_lift(motors, float(d.get("target_pitch_deg", sim.airframe.hover_pitch_deg)), **kw)
                        state.log(f"[rc] channel {ch} high: nose lift started, arm when it reports holding")
                elif last and not high and sim.nose_lift is not None and not link.armed:
                    sim.stop_nose_lift()
                    state.log(f"[rc] channel {ch} low: nose lift stopped")
                last = high
            except Exception as e:  # never let a transmitter glitch kill the watcher
                state.log(f"[rc] switch watcher: {e}")

    import threading
    threading.Thread(target=rc_switch_loop, name="rc-switch", daemon=True).start()

    @app.post("/api/sim/wind")
    async def sim_wind(body: dict):
        sim.set_wind(float(body.get("north", 0)), float(body.get("east", 0)), float(body.get("down", 0)))
        return {"ok": True}

    @app.post("/api/sim/noise")
    async def sim_noise(body: dict):
        sim.sensors.noise.enabled = bool(body.get("enabled", True))
        return {"ok": True}

    @app.post("/api/sim/home")
    async def sim_home(body: dict):
        sim.sensors.set_home(float(body["lat"]), float(body["lon"]), float(body.get("alt", 0.0)))
        return {"ok": True}

    # ------------------------------------------------------------ live scenario on the app's simulator (SITL or HITL)
    @app.post("/api/scenario/start")
    async def scenario_start(body: dict | None = None):
        """Fly a scenario on the live simulator, i.e. on whatever PX4 the app is connected to (the HITL board
        included). The scenario's params are pushed to PX4 first; the vehicle is put back on its legs unless
        reset is false. Progress and the final result come from GET /api/scenario/status."""
        from ..sim.scenario import load_scenario, ScenarioRunner
        from ..sim.metrics import MetricsRecorder
        body = body or {}
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 not connected"}, status_code=409)
        job = state.scenario_job
        if job.get("runner") is not None and not job["runner"].done:
            return JSONResponse({"ok": False, "error": "a scenario is already running"}, status_code=409)
        spec = body.get("scenario", "hover")
        sc = load_scenario(spec if isinstance(spec, dict) else str(spec))
        if body.get("attitude"):
            sc.attitude = dict(sc.attitude or {}); sc.attitude.update(body["attitude"])
        if sc.attitude:
            try:
                base = base_airframe()
                af2 = sc.apply_attitude(base)
                if af2 is not sim.airframe:
                    state.attitude_base = (base, af2.to_dict())
                    sim.set_airframe(af2, keep_state=False)
                    state.log(f"[scenario] {sc.name}: parked at {af2.landed_pitch_deg:g} deg, hover at {af2.hover_pitch_deg:g} deg (legs re-solved)")
            except Exception as e:
                return JSONResponse({"ok": False, "error": f"attitude: {e}"}, status_code=400)
        params = dict(sc.params or {}); params.update(body.get("params") or {})
        pushed = []
        if body.get("push_params", True) and link.params:
            # the airframe's own PX4 parameters first (geometry, rotation, output functions): a headless run seeds
            # them at boot, the live PX4 may still carry another airframe's
            exp = export_params()
            stale = {k: v for k, v in exp.items() if k in link.params
                     and abs(float(link.params[k].get("value", 0.0)) - float(v)) > 1e-4}
            if stale:
                res = await run_in_threadpool(link.set_params, stale)
                bad = [r for r in res if not r.get("ok")]
                state.log(f"[scenario] airframe export: {len(stale) - len(bad)}/{len(stale)} changed parameters pushed"
                          + (f"; FAILED: {[r['name'] for r in bad]}" if bad else ""))
                pushed += bad
        if params and body.get("push_params", True):
            present = {k: v for k, v in params.items() if k in link.params}
            skipped = [k for k in params if k not in link.params]
            res = await run_in_threadpool(link.set_params, present)
            pushed = [r for r in res if not r.get("ok")]
            state.log(f"[scenario] {len(present) - len(pushed)}/{len(present)} parameters pushed for {sc.name}"
                      + (f"; unknown to this firmware: {skipped}" if skipped else "")
                      + (f"; FAILED: {[r['name'] for r in pushed]}" if pushed else ""))
        if body.get("reset", True):
            if getattr(link, "armed", False):
                # a previous flight (or crash) left PX4 armed: it must be disarmed before the vehicle is put back
                await run_in_threadpool(link.disarm, True)
                for _ in range(30):
                    if not link.armed:
                        break
                    await asyncio.sleep(0.1)
            sim.reset(yaw=float(body.get("yaw", 0.0)))
        runner = ScenarioRunner(sc, link, log=state.log, metrics=MetricsRecorder())
        state.scenario_job = {"runner": runner, "name": sc.name, "started": time.time(), "result": None, "failed_params": pushed}
        sim.hooks.append(runner)
        return {"ok": True, "name": sc.name, "phases": [x.get("name") or x.get("type") for x in sc.phases], "failed_params": pushed}

    @app.get("/api/scenario/status")
    async def scenario_status():
        job = state.scenario_job
        r = job.get("runner")
        if r is None:
            return {"running": False}
        out = {"running": not r.done, "name": job.get("name"), "status": r.status, "ok": r.ok, "failures": r.failures,
               "phase": r.phase.get("name") or r.phase.get("type") if r.phase else None, "phase_index": r.index,
               "phase_count": len(r.sc.phases), "sim_time": r.metrics.rows[-1][0] if r.metrics.rows else 0.0,
               "throttle": getattr(r, "last_throttle", None), "events": r.metrics.events[-12:]}
        if r.done:
            if job.get("result") is None:
                try:
                    job["result"] = json_safe(r.result(sim.airframe.mass.mass))
                except Exception as e:
                    job["result"] = {"error": str(e)}
                if r in sim.hooks:
                    sim.hooks.remove(r)
                tune = job.get("tuning")
                if tune:
                    try:
                        res = dict(job["result"]); res.setdefault("scenario", job.get("name"))
                        res["ok"] = bool(r.ok); res["status"] = r.status; res["failures"] = list(r.failures)
                        res["airframe_name"] = sim.airframe.name; res["physics"] = getattr(sim, "physics", "live")
                        tuning.save_run(tune["id"], res, json_safe(r.metrics.timeseries()),
                                        {"name": tune.get("name"), "kind": "live", "params": tune.get("params") or {}})
                        state.log(f"[tuning] live flight saved as {tune['id']}")
                    except Exception as e:
                        state.log(f"[tuning] could not save the live flight: {e}")
            out["result"] = job["result"]
            out["tuning_id"] = (job.get("tuning") or {}).get("id")
        return out

    @app.post("/api/scenario/stop")
    async def scenario_stop():
        job = state.scenario_job
        r = job.get("runner")
        if r is not None and not r.done:
            r.finish(sim, "stopped", False)
            if r in sim.hooks:
                sim.hooks.remove(r)
            state.log("[scenario] stopped by the user")
        return {"ok": True}

    # ------------------------------------------------------------ scenarios / batch / studies
    @app.get("/api/scenarios")
    async def list_scenarios():
        out = []
        for p in sorted(SCENARIO_DIR.glob("*.json")):
            try:
                d = json.loads(p.read_text())
                out.append({"file": p.name, "name": d.get("name", p.stem), "description": d.get("description", ""),
                            "phases": [x.get("name") or x.get("type") for x in d.get("phases", [])],
                            "attitude": d.get("attitude") or {}})
            except Exception:
                pass
        return {"scenarios": out}

    @app.get("/api/studies")
    async def list_studies():
        out = []
        for p in sorted(STUDY_DIR.glob("*.json")):
            try:
                d = json.loads(p.read_text())
                out.append({"file": p.name, "name": d.get("name", p.stem), "description": d.get("description", ""),
                            "variables": d.get("variables", []), "objective": d.get("objective")})
            except Exception:
                pass
        return {"studies": out}

    @app.post("/api/batch/run")
    async def batch_run(body: dict):
        """Headless run of a scenario on the current (or given) airframe in a private PX4 instance. Returns a job id;
        poll /api/batch/jobs. Never touches the interactive simulation."""
        from ..batch.worker import run_once
        scenario = body.get("scenario", "hover")
        af = Airframe.from_dict(body["airframe"]) if body.get("airframe") else base_airframe().copy()
        variables = body.get("variables") or {}
        opts = dict(body.get("options") or {})
        job_id = f"job{int(time.time() * 1000) % 100000000}"
        job = {"id": job_id, "scenario": scenario, "variables": variables, "physics": str(opts.get("physics", "python")), "running": True,
               "t0": time.time(), "result": None, "log": []}
        state.batch_jobs[job_id] = job
        if len(state.batch_jobs) > 50:
            for k in list(state.batch_jobs)[:-50]:
                state.batch_jobs.pop(k, None)

        def run():
            try:
                r = run_once(af, scenario, variables=variables, px4_dir=state.args.px4_dir, log=lambda s: job["log"].append(s),
                             quiet=True, **{k: v for k, v in opts.items() if k in ("speed", "rate", "substeps", "noise", "seed", "timeout_wall", "physics")})
                r.pop("airframe", None)
                job["result"] = r
            except Exception as e:
                job["result"] = {"ok": False, "status": "error", "failures": [str(e)]}
            finally:
                job["running"] = False
                job["t1"] = time.time()

        import threading
        threading.Thread(target=run, name=f"batch-{job_id}", daemon=True).start()
        return {"ok": True, "id": job_id}

    @app.get("/api/batch/jobs")
    async def batch_jobs():
        jobs = []
        for j in state.batch_jobs.values():
            jobs.append({k: v for k, v in j.items() if k != "log"} | {"log": j["log"][-12:]})
        return {"jobs": jobs[::-1], "free_instances": [i for i in range(1, 10) if instance_is_free(i)]}

    @app.post("/api/study/run")
    async def study_run(body: dict):
        from ..batch.study import run_study, load_study
        if state.study_job.get("running"):
            return JSONResponse({"ok": False, "error": "a study is already running"}, status_code=409)
        spec = load_study(body["spec"]) if isinstance(body.get("spec"), str) else dict(body.get("spec") or {})
        if body.get("use_current_airframe", False) or "airframe" not in spec:
            spec["airframe"] = base_airframe().to_dict()
        return start_study(spec, body.get("workers"))

    def start_study(spec: dict, workers=None):
        from ..batch.study import run_study
        job = state.study_job
        job.update(running=True, name=spec.get("name", "study"), trials=[], summary=None, error=None, log=[], t0=time.time(),
                   workers=int(workers or spec.get("workers", 4)))

        def progress(t):
            metrics = t.get("metrics") or {}
            first = next(iter(metrics.values()), {}) if isinstance(metrics, dict) else {}
            row = {k: v for k, v in t.items() if k not in ("metrics", "timing")}
            row["brief"] = tuning.brief({"ok": t.get("ok"), "status": t.get("status"), "failures": t.get("failures"), "metrics": first})
            job["trials"].append(row)

        def run():
            try:
                job["summary"] = run_study(spec, workers=workers, log=lambda s: job["log"].append(s), progress=progress)
            except Exception as e:
                job["error"] = f"{type(e).__name__}: {e}"
            finally:
                job["running"] = False

        import threading
        threading.Thread(target=run, name="study", daemon=True).start()
        return {"ok": True, "name": spec.get("name")}

    @app.get("/api/study/status")
    async def study_status():
        j = state.study_job
        return {"running": j.get("running", False), "name": j.get("name"), "trials": j.get("trials", [])[-200:],
                "summary": j.get("summary"), "error": j.get("error"), "log": j.get("log", [])[-20:]}

    @app.post("/api/airframe/apply_variables")
    async def airframe_apply_variables(body: dict):
        """Apply {path: value} to the live airframe (what a study's 'Apply best' does)."""
        try:
            af = apply_variables(sim.airframe, body.get("variables") or {})
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)
        sim.set_airframe(af, keep_state=True)
        return {"ok": True, "airframe": af.to_dict()}

    # ------------------------------------------------------------ tuning tab
    @app.get("/api/tuning/runs")
    async def tuning_runs():
        return {"runs": tuning.list_runs(), "jobs": [{k: v for k, v in j.items() if k != "log"} | {"log": j["log"][-3:]}
                                                      for j in state.batch_jobs.values() if j.get("tuning")][::-1],
                "free_instances": [i for i in range(1, 10) if instance_is_free(i)]}

    @app.get("/api/tuning/run/{run_id}")
    async def tuning_run(run_id: str, points: int = 1500):
        r = tuning.load_run(run_id, max_points=points)
        if r is None:
            return JSONResponse({"ok": False, "error": "no such run"}, status_code=404)
        return r

    @app.delete("/api/tuning/run/{run_id}")
    async def tuning_run_delete(run_id: str):
        return {"ok": tuning.delete_run(run_id)}

    @app.post("/api/tuning/run")
    async def tuning_start(body: dict):
        """One tuning attempt: the current airframe with ``params`` (PX4 name -> value) on top of its overrides, flown
        through ``scenario``. Headless (a private PX4 instance, saved with its time series when done) or live on the
        app's PX4 (SITL or the HITL board; saved when the scenario ends)."""
        from ..batch.worker import run_once
        scenario = str(body.get("scenario", "stab_lab"))
        params = {str(k): v for k, v in (body.get("params") or {}).items() if str(k).strip()}
        variables = dict(body.get("variables") or {})
        variables.update({f"px4.{k}": v for k, v in params.items()})
        run_id = tuning.new_id("live" if body.get("live") else "run")
        name = str(body.get("name") or run_id)
        attitude = {k: float(v) for k, v in (body.get("attitude") or {}).items() if v is not None and str(v) != ""}
        if attitude:                    # the attempt overrides the scenario's parked / hover pitch
            from ..sim.scenario import load_scenario
            sc_d = load_scenario(scenario).to_dict()
            sc_d["attitude"] = dict(sc_d.get("attitude") or {}); sc_d["attitude"].update(attitude)
            scenario = sc_d
        if body.get("live"):
            r = await scenario_start({"scenario": scenario, "params": params, "reset": body.get("reset", True)})
            if isinstance(r, JSONResponse):
                return r
            state.scenario_job["tuning"] = {"id": run_id, "name": name, "params": params}
            return {"ok": True, "id": run_id, "live": True, "phases": r.get("phases")}
        af = base_airframe().copy()
        opts = dict(body.get("options") or {})
        # reserve a PX4 instance now: two attempts started in the same second would otherwise both pick the first
        # free one and collide on its ports
        taken = {j.get("instance") for j in state.batch_jobs.values() if j.get("running")}
        if state.study_job.get("running"):
            # a running sweep cycles PX4 on the lowest instances between trials; they look free for a moment
            taken |= set(range(1, int(state.study_job.get("workers", 4)) + 1))
        instance = next((i for i in range(9, 0, -1) if i not in taken and instance_is_free(i)), None)
        if instance is None:
            return JSONResponse({"ok": False, "error": "no free PX4 instance (1..9)"}, status_code=409)
        job = {"id": run_id, "name": name, "tuning": True, "instance": instance, "scenario": scenario if isinstance(scenario, str) else scenario.get("name"),
               "variables": variables, "params": params, "attitude": attitude,
               "physics": str(opts.get("physics", "python")), "running": True, "t0": time.time(), "result": None, "log": []}
        state.batch_jobs[run_id] = job

        def run():
            tuning.TUNING_DIR.mkdir(parents=True, exist_ok=True)
            ts_path = tuning.TUNING_DIR / f"{run_id}_ts.json"
            try:
                r = run_once(af, scenario, variables=variables, px4_dir=state.args.px4_dir, log=lambda s: job["log"].append(s),
                             quiet=True, timeseries_path=str(ts_path), task_id=run_id, instance=instance,
                             **{k: v for k, v in opts.items() if k in ("speed", "rate", "substeps", "noise", "seed", "timeout_wall", "physics")})
                r.pop("airframe", None)
                ts = None
                if ts_path.is_file():
                    ts = json.loads(ts_path.read_text()).get("timeseries")
                tuning.save_run(run_id, r, ts, {"name": name, "kind": "headless", "params": params})
                job["result"] = r
            except Exception as e:
                job["result"] = {"ok": False, "status": "error", "failures": [str(e)]}
                tuning.save_run(run_id, job["result"], None, {"name": name, "kind": "headless", "params": params, "scenario": scenario})
            finally:
                job["running"] = False
                job["t1"] = time.time()

        import threading
        threading.Thread(target=run, name=f"tuning-{run_id}", daemon=True).start()
        return {"ok": True, "id": run_id, "live": False}

    @app.get("/api/tuning/sweeps")
    async def tuning_sweeps():
        j = state.study_job
        return {"sweeps": tuning.list_sweeps(), "running": bool(j.get("running")), "current": j.get("name"),
                "log": (j.get("log") or [])[-6:], "error": j.get("error")}

    @app.get("/api/tuning/sweep/{study}/trial/{k}")
    async def tuning_trial(study: str, k: int, points: int = 1500):
        r = tuning.load_trial(study, k, max_points=points)
        if r is None:
            return JSONResponse({"ok": False, "error": "no such trial"}, status_code=404)
        return r

    @app.post("/api/tuning/sweep")
    async def tuning_sweep(body: dict):
        """Build and start a grid sweep over PX4 parameters on the current airframe: body {name, scenario, variables:
        [{param, min, max, levels}], workers, base_variables, objective, constraints}. Trials keep their time series."""
        if state.study_job.get("running"):
            return JSONResponse({"ok": False, "error": "a study is already running"}, status_code=409)
        variables = body.get("variables") or []
        if not variables:
            return JSONResponse({"ok": False, "error": "no variables"}, status_code=400)
        workers = int(body.get("workers") or 4)
        spec = tuning.sweep_spec(base_airframe().to_dict(), str(body.get("scenario", "stab_lab")), variables, name=body.get("name"),
                                 workers=workers, base_variables=body.get("base_variables"), objective=body.get("objective") or None,
                                 constraints=body.get("constraints") or None, algorithm=body.get("algorithm"))
        return start_study(spec, workers)

    @app.get("/api/tuning/defaults")
    async def tuning_defaults():
        gains = ["MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D",
                 "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P", "MC_YAW_WEIGHT", "MC_YAWRATE_MAX",
                 "CA_METHOD", "MC_AIRMODE", "MPC_THR_HOVER", "THR_MDL_FAC"]
        ov = sim.airframe.px4_overrides or {}
        cur = {}
        for g in gains:
            if g in ov:
                cur[g] = ov[g]
            elif g in (link.params or {}):
                cur[g] = link.params[g].get("value")
        return {"params": cur, "objective": tuning.TUNING_OBJECTIVE, "constraints": tuning.TUNING_CONSTRAINTS,
                "overrides": ov}

    # ------------------------------------------------------------ websocket
    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        last_log = 0.0
        last_status = 0.0

        async def receive_loop():
            """Client -> server: joystick frames (and nothing else for now)."""
            while True:
                raw = await websocket.receive_text()
                try:
                    m = json.loads(raw)
                except Exception:
                    continue
                if m.get("type") == "manual" and link.ctl_connected:
                    try:
                        await run_in_threadpool(link.send_manual_control, float(m.get("roll", 0)), float(m.get("pitch", 0)),
                                                float(m.get("throttle", 0)), float(m.get("yaw", 0)), int(m.get("buttons", 0)),
                                                [float(v) for v in (m.get("aux") or [])])
                    except Exception as e:
                        state.log(f"[joystick] send failed: {e}")

        rx_task = asyncio.create_task(receive_loop())
        try:
            await websocket.send_text(json.dumps({"type": "airframe", "airframe": sim.airframe.to_dict()}))
            while True:
                now = time.time()
                msg: dict[str, Any] = {"type": "state", "state": sim.snapshot()}
                if now - last_status > 0.5:
                    msg["status"] = status_dict()
                    last_status = now
                new_logs = [e for e in state.log_buffer if e[0] > last_log]
                if new_logs:
                    msg["log"] = new_logs
                    last_log = new_logs[-1][0]
                await websocket.send_text(json.dumps(msg))
                await asyncio.sleep(1 / 30)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
        finally:
            rx_task.cancel()

    return app
