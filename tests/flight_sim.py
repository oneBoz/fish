"""
Bench flight with the simulated fish, no server and no hardware.

    .venv/bin/python tests/flight_sim.py                 # default mission (2, 8, 1) at 0.8 m/s
    .venv/bin/python tests/flight_sim.py 0 6 0 1.0       # target x y z [speed]
    .venv/bin/python tests/flight_sim.py --abort         # abort 4 s into the glide

Runs the full operator sequence (connect, mission, calibrate, self-test, arm, launch)
through GroundStation.command, prints one line per half second with the TRUE fish
position next to the estimate, and exits 0 only if the fish arrives (or, with
--abort, if the abort lands in the Aborted phase with the fins neutral).
"""
import argparse
import math
import os
import sys
import threading
import time

import numpy as np

sys.stdout.reconfigure(line_buffering=True)   # os._exit below skips the stdio flush
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from gcs.station import GroundStation  # noqa: E402


def run(target, speed, abort_at=None, timeout=45.0, quiet=False):
    args = argparse.Namespace(fake=True, fin_ip="x", cam_ip="x", yolo=None, imu_axes="x,y,z",
                              guidance="laptop", host="127.0.0.1", port=0, open=False)
    st = GroundStation(args)
    st.start()
    for c in ({"cmd": "connect"}, {"cmd": "mission", "speed": speed, "target": target}, {"cmd": "calibrate"}):
        r = st.command(c)
        assert r["ok"], r
    time.sleep(st.cfg["calib_s"] + 0.4)
    assert st.command({"cmd": "selftest"})["ok"]
    time.sleep(2.6)
    assert st.command({"cmd": "arm"})["ok"] and st.command({"cmd": "arm"})["ok"] and st.phase == "armed"
    assert st.command({"cmd": "launch"})["ok"]
    fish = st.link
    t0 = time.time()
    worst = 0.0
    aborted_ok = False
    while time.time() - t0 < timeout:
        t = st.telemetry
        with fish.lock:
            tp = fish.pos.copy()
            rel = fish.target - fish.pos
        d = float(np.linalg.norm(rel))
        e = t["est"]
        perr = float(np.linalg.norm(tp - np.array([e["x"], e["y"], e["z"]])))
        worst = max(worst, perr)
        if not quiet:
            print("%5.1f %-10s prog=%.2f true=(%.2f,%.2f,%.2f) est=(%.2f,%.2f,%.2f) err=%.2f dist=%.2f cv=%-9s conf=%.2f fins=(%.0f,%.0f)" % (
                time.time() - t0, t["phase"], t["progress"], *tp, e["x"], e["y"], e["z"], perr, d,
                t["cv"]["state"], t["cv"]["conf"], t["fins"]["yaw_cmd"], t["fins"]["pitch_cmd"]))
        if abort_at is not None and t["phase"] == "imu_glide" and time.time() - t0 >= abort_at:
            r = st.command({"cmd": "abort"})
            time.sleep(0.3)
            aborted_ok = r["ok"] and st.phase == "aborted" and st.fin_cmd == (0.0, 0.0)
            r2 = st.command({"cmd": "new_mission"})
            print("abort ->", r, "phase", st.phase, "fins", st.fin_cmd, "| new_mission ->", r2, "phase", st.phase)
            break
        if t["phase"] in ("arrived", "aborted"):
            break
        time.sleep(0.5)
    ok = aborted_ok if abort_at is not None else (st.phase == "arrived" and d < 0.6)
    print("RESULT %s | target %s at %.1f m/s | phase %s | true distance %.2f m | worst estimate error %.2f m | %.1f s" % (
        "PASS" if ok else "FAIL", target, speed, st.phase, d, worst, time.time() - t0))
    st.stop()
    return ok


if __name__ == "__main__":
    watchdog = threading.Timer(120, lambda: os._exit(3)); watchdog.daemon = True; watchdog.start()
    argv = [a for a in sys.argv[1:] if not a.startswith("--")]
    nums = [float(a) for a in argv]
    target = nums[:3] if len(nums) >= 3 else [2.0, 8.0, 1.0]
    speed = nums[3] if len(nums) >= 4 else 0.8
    ok = run(target, speed, abort_at=4.0 if "--abort" in sys.argv else None, quiet="--quiet" in sys.argv)
    os._exit(0 if ok else 1)
