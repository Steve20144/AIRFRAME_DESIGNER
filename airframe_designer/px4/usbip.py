"""Attach a USB flight controller to WSL from inside WSL, so HITL works on Windows.

WSL2 has no USB of its own. A Pixhawk plugged into a Windows host is a COM port on the Windows side and simply
does not exist in the distribution, so the Connect tab finds nothing and there is no hint as to why. The bridge is
usbipd-win: Windows shares the device over USB/IP and WSL claims it, after which it appears as /dev/ttyACM*.

That is a Windows-side action, but WSL can run Windows executables directly, so the app can do it for you rather
than leaving you to find the incantation. Attaching needs no elevation as long as the device has already been
bound; binding does, once per device, and this module reports that rather than pretending it can do it.

Everything here no-ops off WSL, where the USB device is already visible to whoever asked.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

#: `usbipd list` prints: busid, vid:pid, description, then the state, separated by runs of spaces.
_ROW = re.compile(r"^\s*(\d+-\d+)\s+([0-9a-fA-F]{4}):([0-9a-fA-F]{4})\s+(.+?)\s{2,}(\S.*?)\s*$")

#: Where usbipd-win installs. `usbipd.exe` on PATH is tried first, for a non-default install.
_CANDIDATES = (
    "/mnt/c/Program Files/usbipd-win/usbipd.exe",
    "/mnt/c/Program Files (x86)/usbipd-win/usbipd.exe",
)


def under_wsl() -> bool:
    if os.environ.get("WSL_DISTRO_NAME"):
        return True
    try:
        return "microsoft" in Path("/proc/version").read_text().lower()
    except OSError:
        return False


def usbipd_path() -> str | None:
    found = shutil.which("usbipd.exe") or shutil.which("usbipd")
    if found:
        return found
    return next((p for p in _CANDIDATES if Path(p).is_file()), None)


def _run(exe: str, *args: str, timeout: float = 20.0) -> tuple[int, str]:
    try:
        p = subprocess.run([exe, *args], capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        return 1, f"{type(e).__name__}: {e}"
    return p.returncode, ((p.stdout or "") + (p.stderr or "")).replace("\r", "")


def _looks_like_px4(vid: int, description: str) -> bool:
    from .connection import PX4_HINTS, PX4_VENDORS
    return vid in PX4_VENDORS or bool(PX4_HINTS.search(description))


def list_devices() -> list[dict]:
    """Every USB device Windows can share, with its busid and whether WSL already has it.

    Windows names a flight controller "USB Serial Device (COMn)" until something reads its descriptors, so the
    vendor id is what identifies it here; the friendly product string only appears once Linux has the device.
    """
    exe = usbipd_path()
    if not exe:
        return []
    code, out = _run(exe, "list")
    if code != 0:
        return []
    devices = []
    for line in out.splitlines():
        if line.strip().lower().startswith("persisted:"):
            break
        m = _ROW.match(line)
        if not m:
            continue
        busid, vid_s, pid_s, description, statev = m.groups()
        vid, pid = int(vid_s, 16), int(pid_s, 16)
        state = statev.strip().lower()
        devices.append({
            "busid": busid, "vid": vid, "pid": pid, "description": description.strip(),
            "state": statev.strip(),
            "attached": state.startswith("attached"),
            "shared": state.startswith("shared") or state.startswith("attached"),
            "likely_px4": _looks_like_px4(vid, description),
        })
    return devices


def status() -> dict:
    """What the Connect tab needs to decide whether to offer the button, and what to say on it."""
    if not under_wsl():
        return {"available": False, "reason": "not running under WSL; USB is already visible here"}
    exe = usbipd_path()
    if not exe:
        return {"available": False, "reason": "usbipd-win is not installed on the Windows host",
                "hint": "install it with:  winget install usbipd"}
    devices = list_devices()
    candidates = [d for d in devices if d["likely_px4"]]
    attached = [d for d in candidates if d["attached"]]
    return {
        "available": True, "usbipd": exe, "devices": devices, "candidates": candidates,
        "attached": bool(attached),
        "needs_bind": bool(candidates) and not any(d["shared"] for d in candidates),
        "summary": (f"{candidates[0]['description']} on {candidates[0]['busid']} "
                    f"({'attached' if attached else candidates[0]['state'].lower()})" if candidates else
                    "no flight controller found on the Windows host"),
    }


def attach(busid: str | None = None) -> dict:
    """Hand the flight controller to WSL. Returns ``ok`` and a line fit to show the operator.

    ``usbipd attach`` needs no elevation, but it only works on a device that has been bound, and binding does.
    A device nobody has bound therefore comes back with the exact command to run, rather than a failure.
    """
    st = status()
    if not st.get("available"):
        return {"ok": False, "message": st.get("reason", "cannot attach here"), "hint": st.get("hint")}
    exe = st["usbipd"]

    if busid:
        target = next((d for d in st["devices"] if d["busid"] == busid), None)
    else:
        target = next((d for d in st["candidates"] if not d["attached"]), None) or \
                 next(iter(st["candidates"]), None)
    if target is None:
        return {"ok": False, "message": "No flight controller found on the Windows host. Is it plugged in?"}

    if target["attached"]:
        return {"ok": True, "message": f"{target['description']} is already attached to WSL ({target['busid']}).",
                "busid": target["busid"]}

    if not target["shared"]:
        # Binding is the one step that needs an administrator, and it is once per device, not once per session.
        return {"ok": False, "busid": target["busid"],
                "message": (f"{target['description']} ({target['busid']}) has not been shared yet. "
                            f"Binding needs an administrator, once."),
                "hint": f"In an admin PowerShell:  usbipd bind --busid {target['busid']}"}

    code, out = _run(exe, "attach", "--wsl", "--busid", target["busid"], timeout=40.0)
    if code != 0:
        return {"ok": False, "busid": target["busid"],
                "message": f"usbipd attach failed for {target['busid']}.", "detail": out.strip()[:400]}
    return {"ok": True, "busid": target["busid"],
            "message": f"Attached {target['description']} ({target['busid']}) to WSL. "
                       f"It is no longer visible to Windows until detached.",
            "detail": out.strip()[:400]}


_auto_attach: subprocess.Popen | None = None


def auto_attach_running() -> bool:
    """A `usbipd attach --auto-attach` is alive: ours, or one started by hand or by an earlier app run."""
    if _auto_attach is not None and _auto_attach.poll() is None:
        return True
    try:
        out = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    return any("usbipd" in line and "--auto-attach" in line for line in out.splitlines())


def ensure_auto_attach(log=None) -> dict:
    """Keep the flight controller attached to WSL through re-enumerations. Flashing reboots the board into its
    bootloader, which is a new USB device to Windows: without --auto-attach it stays on Windows and the uploader
    waits forever. The process is left running (the board also re-enumerates on every parameter-driven reboot);
    it ends with the app."""
    global _auto_attach
    if not under_wsl():
        return {"ok": True, "message": "not WSL: the board is visible directly"}
    if auto_attach_running():
        return {"ok": True, "message": "usbipd auto-attach already running"}
    st = status()
    if not st.get("available"):
        return {"ok": False, "message": st.get("reason", "usbipd not available"), "hint": st.get("hint")}
    target = next((d for d in st["candidates"] if d["attached"]), None) or next(iter(st["candidates"]), None)
    if target is None:
        return {"ok": False, "message": "No flight controller found on the Windows host. Is it plugged in?"}
    if not target["shared"]:
        return {"ok": False, "busid": target["busid"], "message": f"{target['busid']} has not been shared yet.",
                "hint": f"In an admin PowerShell:  usbipd bind --busid {target['busid']}"}
    _auto_attach = subprocess.Popen([st["usbipd"], "attach", "--wsl", "--busid", target["busid"], "--auto-attach"],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    if log is not None:
        import threading

        def pump(proc=_auto_attach):
            for line in proc.stdout:
                line = line.replace("\r", "").strip()
                if line:
                    log(f"[usb] {line}")
        threading.Thread(target=pump, daemon=True).start()
    return {"ok": True, "busid": target["busid"],
            "message": f"usbipd auto-attach started for {target['description']} ({target['busid']})"}


def detach(busid: str | None = None) -> dict:
    """Give the device back to Windows, so QGroundControl there can see it again."""
    st = status()
    if not st.get("available"):
        return {"ok": False, "message": st.get("reason", "cannot detach here")}
    target = busid or next((d["busid"] for d in st["candidates"] if d["attached"]), None)
    if not target:
        return {"ok": False, "message": "nothing attached to detach"}
    code, out = _run(st["usbipd"], "detach", "--busid", target)
    return {"ok": code == 0, "busid": target, "detail": out.strip()[:400],
            "message": (f"Detached {target}; Windows has the device again." if code == 0
                        else f"usbipd detach failed for {target}.")}
