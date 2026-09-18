"""Handing a USB flight controller from Windows to WSL.

None of this needs usbipd, WSL or a board: the parsing and the decisions are pure, and they are what goes wrong.
The interesting cases are the ones an operator hits and cannot diagnose from the UI, so each has a test: a device
nobody has bound (which needs an administrator once), a device already attached (clicking twice must not be an
error), and running somewhere that has USB of its own (the button must not appear at all).
"""
from __future__ import annotations

import pytest

from airframe_designer.px4 import usbip

# Real `usbipd list` output, including the trailing section that must not be parsed as devices.
LISTING = """Connected:
BUSID  VID:PID    DEVICE                                                        STATE
1-2    046d:c542  USB Input Device                                              Not shared
1-4    3185:0035  USB Serial Device (COM3)                                      Shared
1-7    3277:0029  USB2.0 HD UVC WebCam                                          Not shared
1-10   8087:0026  Intel(R) Wireless Bluetooth(R)                                Not shared

Persisted:
GUID                                  DEVICE
abcd1234-0000-0000-0000-00000000beef  Some Detached Thing
"""


def _with_state(listing: str, busid: str, state: str) -> str:
    """Rewrite one device's STATE column. Done by line rather than by matching the exact run of spaces, which is
    what the column happens to be padded to and is not worth encoding in a test."""
    out = []
    for line in listing.splitlines():
        m = usbip._ROW.match(line)
        if m and m.group(1) == busid:
            line = line[: line.rstrip().rfind(m.group(5))] + state
        out.append(line)
    return "\n".join(out) + "\n"


@pytest.fixture
def fake_usbipd(monkeypatch):
    """A usbipd that reports LISTING and records what it was asked to do."""
    calls: list[tuple[str, ...]] = []
    listing = {"text": LISTING}

    def run(exe, *args, timeout=20.0):
        calls.append(args)
        if args and args[0] == "list":
            return 0, listing["text"]
        return 0, "ok"

    monkeypatch.setattr(usbip, "under_wsl", lambda: True)
    monkeypatch.setattr(usbip, "usbipd_path", lambda: "/mnt/c/Program Files/usbipd-win/usbipd.exe")
    monkeypatch.setattr(usbip, "_run", run)
    return {"calls": calls, "listing": listing}


def test_listing_is_parsed_and_the_persisted_section_ignored(fake_usbipd):
    devices = usbip.list_devices()
    assert [d["busid"] for d in devices] == ["1-2", "1-4", "1-7", "1-10"]
    board = next(d for d in devices if d["busid"] == "1-4")
    assert (board["vid"], board["pid"]) == (0x3185, 0x0035)
    assert board["shared"] and not board["attached"]


def test_the_flight_controller_is_found_by_vendor_id_not_by_its_name(fake_usbipd):
    """Windows calls it "USB Serial Device (COM3)" until something reads its descriptors, so the name is no help
    and the vendor id is the whole basis for picking it out of a webcam, a mouse and a Bluetooth radio."""
    candidates = [d for d in usbip.list_devices() if d["likely_px4"]]
    assert [d["busid"] for d in candidates] == ["1-4"]


def test_attaching_a_shared_device_calls_usbipd_and_reports_the_windows_side_effect(fake_usbipd):
    r = usbip.attach()
    assert r["ok"] and r["busid"] == "1-4"
    assert ("attach", "--wsl", "--busid", "1-4") in fake_usbipd["calls"]
    assert "no longer visible to Windows" in r["message"]


def test_attaching_twice_is_not_an_error(fake_usbipd):
    fake_usbipd["listing"]["text"] = _with_state(LISTING, "1-4", "Attached")
    r = usbip.attach()
    assert r["ok"] and "already attached" in r["message"]
    assert not any(a and a[0] == "attach" for a in fake_usbipd["calls"])


def test_an_unbound_device_asks_for_the_one_admin_command_instead_of_failing(fake_usbipd):
    """Binding needs an administrator and attaching does not. Telling the operator to run the wrong one, or just
    saying it failed, is the difference between a 20 second fix and an afternoon."""
    fake_usbipd["listing"]["text"] = _with_state(LISTING, "1-4", "Not shared")
    r = usbip.attach()
    assert not r["ok"]
    assert "usbipd bind --busid 1-4" in r["hint"]
    assert not any(a and a[0] == "attach" for a in fake_usbipd["calls"])


def test_nothing_plugged_in_says_so(fake_usbipd):
    fake_usbipd["listing"]["text"] = "\n".join(
        l for l in LISTING.splitlines() if "3185:0035" not in l)
    r = usbip.attach()
    assert not r["ok"] and "plugged in" in r["message"]


def test_off_wsl_the_feature_is_unavailable_so_the_button_stays_hidden(monkeypatch):
    monkeypatch.setattr(usbip, "under_wsl", lambda: False)
    st = usbip.status()
    assert st["available"] is False and "WSL" in st["reason"]
    assert usbip.attach()["ok"] is False


def test_without_usbipd_installed_the_hint_is_how_to_install_it(monkeypatch):
    monkeypatch.setattr(usbip, "under_wsl", lambda: True)
    monkeypatch.setattr(usbip, "usbipd_path", lambda: None)
    st = usbip.status()
    assert st["available"] is False and "winget install usbipd" in st["hint"]


def test_status_summarises_what_the_button_should_say(fake_usbipd):
    st = usbip.status()
    assert st["available"] and not st["attached"] and not st["needs_bind"]
    assert "1-4" in st["summary"]
