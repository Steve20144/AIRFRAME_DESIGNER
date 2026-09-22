"""The Flash tab's job runner: every line kept for the live log, build and upload progress parsed."""
import time

from airframe_designer.px4 import firmware_images as fi
from airframe_designer.px4.connection import FirmwareJob


def test_job_keeps_lines_and_parses_progress(tmp_path):
    script = tmp_path / "fake.sh"
    script.write_text("echo '[5/10] Building CXX object a.o'\n"
                      "printf 'Program: [==========          ] 50.0%%\\rProgram: [====================] 100.0%%\\n'\n"
                      "echo done\n")
    logged: list[str] = []
    job = FirmwareJob(logged.append)
    assert job.start("px4_fmu-v6x", "upload", str(tmp_path), "/usr/bin", script=script, script_args=[],
                     image="nose_lift", label="flash test")["ok"]
    t0 = time.time()
    while job.running() and time.time() - t0 < 10:
        time.sleep(0.05)
    time.sleep(0.2)
    st = job.status()
    assert st["result"] == "ok" and st["image"] == "nose_lift" and st["progress"]["phase"] == "done"
    texts = [t for _, t in job.lines_since(0)["lines"]]
    assert "[5/10] Building CXX object a.o" in texts
    assert any(t.startswith("Program:") and t.endswith("50.0%") for t in texts)     # \r redraws arrive as lines
    assert texts[-1].startswith("[flash test finished (exit 0)")
    nxt = job.lines_since(0)["next"]
    assert job.lines_since(nxt)["lines"] == []


def test_images_describe_their_build():
    for img in fi.IMAGES:
        d = fi.describe(img, "px4_fmu-v6x")
        assert d["file"].endswith("px4_fmu-v6x_" + d["variant"] + ".px4")
        script, args, _ = fi.script_call(img, "px4_fmu-v6x", "upload")
        assert script.is_file() and "upload" in args
