"""The flight-controller firmware images the Flash tab builds and flashes.

Each image is a PX4 tree plus the script that builds (and uploads) it for a board:

* ``nose_lift``: PX4 v1.17.0 with the HIL output driver, the nose_lift module and the control-allocator hook
  (``firmware/``), built in the dedicated worktree ``~/PX4-nl`` by ``scripts/build_nose_lift_firmware.sh``.
* ``hitl``: the stock v1.17.0 image with only the HIL output driver added, from ``~/PX4-hitl`` via
  ``scripts/build_hitl_firmware.sh``: what the board ran before, kept so it can go back.
"""
from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_TARGET = "px4_fmu-v6x"

IMAGES: list[dict] = [
    {"id": "nose_lift", "name": "Nose lift",
     "detail": "PX4 v1.17.0 + HIL output driver + the nose_lift module and its control-allocator hook. Arm first, "
               "then the RC switch; the kill switch stops it in every phase.",
     "px4_dir": "~/PX4-nl", "script": "scripts/build_nose_lift_firmware.sh",
     "sources": ["firmware", "scripts/build_nose_lift_firmware.sh"]},
    {"id": "hitl", "name": "HITL (stock)",
     "detail": "PX4 v1.17.0 + HIL output driver only, no nose lift: the image the board ran before.",
     "px4_dir": "~/PX4-hitl", "script": "scripts/build_hitl_firmware.sh", "sources": []},
]


def get(image_id: str) -> dict | None:
    return next((i for i in IMAGES if i["id"] == image_id), None)


def px4_dir(img: dict) -> str:
    return os.path.expanduser(img["px4_dir"])


def variant(img: dict, target: str) -> str:
    """'multicopter' when the board offers it (the fmu-v6x default image is full), else 'default'."""
    board_dir = Path(px4_dir(img)) / "boards" / target.replace("_", "/", 1)
    return "multicopter" if (board_dir / "multicopter.px4board").is_file() else "default"


def image_file(img: dict, target: str) -> Path:
    v = variant(img, target)
    return Path(px4_dir(img)) / "build" / f"{target}_{v}" / f"{target}_{v}.px4"


def script_call(img: dict, target: str, action: str) -> tuple[Path, list[str], dict]:
    """(script, arguments, extra environment) for 'build' or 'upload'."""
    script = PROJECT_DIR / img["script"]
    v = variant(img, target)
    if img["id"] == "nose_lift":
        return script, ["board", action], {"BOARD": target, "VARIANT": v}
    return script, [target, action, v], {}


def _newest_source(img: dict) -> float:
    newest = 0.0
    for rel in img.get("sources") or []:
        p = PROJECT_DIR / rel
        files = [p] if p.is_file() else [f for f in p.rglob("*") if f.is_file()] if p.is_dir() else []
        for f in files:
            newest = max(newest, f.stat().st_mtime)
    return newest


def _git_describe(path: str) -> str:
    try:
        r = subprocess.run(["git", "-C", path, "describe", "--tags", "--always", "--dirty"], capture_output=True,
                           text=True, timeout=5)
        return r.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def describe(img: dict, target: str) -> dict:
    tree = px4_dir(img)
    f = image_file(img, target)
    out = {k: img[k] for k in ("id", "name", "detail", "px4_dir")}
    out.update({"target": target, "variant": variant(img, target), "tree_exists": Path(tree).is_dir(),
                "file": str(f), "built": f.is_file(), "px4_ref": _git_describe(tree) if Path(tree).is_dir() else ""})
    if f.is_file():
        st = f.stat()
        newest = _newest_source(img)
        out.update({"size": st.st_size, "mtime": st.st_mtime, "age_s": round(time.time() - st.st_mtime),
                    "stale": newest > st.st_mtime + 1.0})
    return out
