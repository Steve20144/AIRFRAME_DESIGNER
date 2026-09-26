"""List, fetch and upload files on the flight controller's SD card over MAVLink FTP (Windows: the board's COM port).

  python scripts/board_sdcard.py ls   --port COM3 /fs/microsd/etc/logging
  python scripts/board_sdcard.py get  --port COM3 /fs/microsd/etc/logging/logger_topics.txt local.txt
  python scripts/board_sdcard.py put  --port COM3 firmware/sdcard/etc/logging/logger_topics.txt /fs/microsd/etc/logging/
  python scripts/board_sdcard.py put  ... --reboot     after the upload, reboot the board (refused while armed)

`put` saves any file it replaces to results/board_sdcard/ first, creates missing directories, and reads the upload
back to check it byte for byte. Close QGC first (it holds the port). USB only in practice: FTP over the SiK radio
works but is slow and drops packets. From Git Bash set MSYS_NO_PATHCONV=1, or it rewrites /fs/... into a Windows path.
"""
import argparse
import logging
import os
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402
from pymavlink.mavftp import MAVFTP, FtpError  # noqa: E402
from pymavlink.mavftp_op import FTP_OP, OP_ResetSessions  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "results" / "board_sdcard"


def connect(port):
    m = mavutil.mavlink_connection(port, baud=115200, source_system=254)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        sys.exit(f"no heartbeat on {port}")
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"{port}: system {m.target_system}, {'ARMED' if armed else 'disarmed'}")
    return m, armed


def open_ftp(m):
    """MAVFTP that works on Windows, with the board's FTP sessions cleared first."""
    ftp = MAVFTP(m, m.target_system, m.target_component)
    # pymavlink downloads into a fixed /tmp/temp_mavftp_file, which does not exist on Windows; a failed download
    # also leaves the board's only FTP session open ("no sessions available" until a reset or a reboot)
    ftp.temp_filename = str(Path(tempfile.gettempdir()) / "temp_mavftp_file")
    reset_sessions(ftp)
    return ftp


def reset_sessions(ftp):
    """Free every FTP session on the board (a failed or unterminated transfer keeps PX4's only one busy). PX4 answers a
    request whose sequence number is one below its last reply's with that old reply, so a reset can be swallowed;
    two resets with numbers far apart cannot both be."""
    for jump in (97, 61):
        ftp.seq = (ftp.seq + jump) % 256
        ftp._MAVFTP__send(FTP_OP(ftp.seq, 0, OP_ResetSessions, 0, 0, 0, 0, None))
        ftp.process_ftp_reply("ResetSessions", timeout=5)


def ok(ftp, ret, op, timeout=10):
    if ret.error_code == FtpError.Success:
        ret = ftp.process_ftp_reply(op, timeout=timeout)
    return ret.error_code == FtpError.Success


def entries(ftp, path):
    """{name: (is_dir, size)} of a directory; empty if it is empty or missing. pymavlink ends every listing with a
    generic Fail code, so existence is decided by listing the parent (see is_dir)."""
    if ftp.cmd_list([path]).error_code == FtpError.Success:
        ftp.process_ftp_reply("ListDirectory", timeout=10)
    return {e.name: (e.is_dir, e.size_b) for e in ftp.list_result if e.name not in (".", "..")}


def is_dir(ftp, path):
    parent, name = path.rstrip("/").rsplit("/", 1)
    return path == "/" or entries(ftp, parent or "/").get(name, (False,))[0]


def listing(ftp, path):
    """{name: size} of a directory, or None if it does not exist."""
    if not is_dir(ftp, path):
        return None
    return {k: size for k, (_, size) in entries(ftp, path).items()}


def fetch(ftp, remote, local):
    local = Path(local)
    local.unlink(missing_ok=True)
    ok(ftp, ftp.cmd_get([remote, str(local)]), "OpenFileRO", timeout=30)
    return local.exists()


def cmd_ls(a):
    m, _ = connect(a.port)
    entries = listing(open_ftp(m), a.path)
    if entries is None:
        sys.exit(f"{a.path}: not found")
    for name, size in sorted(entries.items()):
        print(f"{size:>10}  {name}")


def cmd_get(a):
    m, _ = connect(a.port)
    if not fetch(open_ftp(m), a.remote, a.local):
        sys.exit(f"could not read {a.remote}")
    print(f"{a.remote} -> {a.local}")


def cmd_put(a):
    m, armed = connect(a.port)
    ftp = open_ftp(m)
    local = Path(a.local)
    remote = a.remote + local.name if a.remote.endswith("/") else a.remote
    folder, name = remote.rsplit("/", 1)

    # create missing directories one level at a time
    parts = folder.strip("/").split("/")
    for i in range(1, len(parts) + 1):
        d = "/" + "/".join(parts[:i])
        if not is_dir(ftp, d):
            ok(ftp, ftp.cmd_mkdir([d]), "CreateDirectory")
            if not is_dir(ftp, d):
                sys.exit(f"could not create {d}")
            print(f"created {d}")

    if name in (listing(ftp, folder) or {}):
        OUT.mkdir(parents=True, exist_ok=True)
        backup = OUT / f"{time.strftime('%Y%m%d_%H%M%S')}_{name}"
        if not fetch(ftp, remote, backup):
            sys.exit(f"{remote} exists but could not be backed up; nothing changed")
        print(f"old {remote} saved to {backup}")

    # a fresh MAVFTP per transfer: after a download, pymavlink's put on the same object truncates the file and writes
    # nothing (seen 2026-09-23), and a get right after a put finds the session still busy
    ftp = open_ftp(m)
    ok(ftp, ftp.cmd_put([str(local), remote]), "CreateFile", timeout=30)   # judged by the read-back below
    with tempfile.TemporaryDirectory() as tmp:
        check = Path(tmp) / name
        if not fetch(open_ftp(m), remote, check) or check.read_bytes() != local.read_bytes():
            sys.exit(f"read-back of {remote} does not match {local}")
    print(f"{local} -> {remote} ({local.stat().st_size} bytes, read back and identical)")

    if a.reboot:
        if armed:
            sys.exit("board is armed: not rebooting")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        print("reboot sent")


def main():
    logging.basicConfig(level=logging.WARNING)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("ls"); p.add_argument("--port", required=True); p.add_argument("path")
    p = sub.add_parser("get"); p.add_argument("--port", required=True); p.add_argument("remote"); p.add_argument("local")
    p = sub.add_parser("put"); p.add_argument("--port", required=True); p.add_argument("local"); p.add_argument("remote")
    p.add_argument("--reboot", action="store_true")
    a = ap.parse_args()
    {"ls": cmd_ls, "get": cmd_get, "put": cmd_put}[a.cmd](a)


if __name__ == "__main__":
    main()
