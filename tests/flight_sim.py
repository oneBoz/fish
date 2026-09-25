"""
Bench flight with the simulated fish, no server and no hardware.

    .venv/bin/python tests/flight_sim.py                 # default mission (2, 8, 1) at 0.8 m/s
    .venv/bin/python tests/flight_sim.py 0 6 0 1.0       # target x y z [speed]
    .venv/bin/python tests/flight_sim.py --abort         # abort 4 s into the glide
    .venv/bin/python tests/flight_sim.py --stop-restart  # arm without checks, stop mid-glide, re-apply, fly again
    .venv/bin/python tests/flight_sim.py --swarm         # swarm vision mode: 5 simulated quads, home on their centroid

Runs the full operator sequence (connect, mission, calibrate, self-test, arm, launch)
through GroundStation.command, prints one line per half second with the TRUE fish
position next to the estimate, and exits 0 only if the fish arrives (or, with
--abort, if the abort lands back in Ready with the fins neutral and the history kept).
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


def run_stop_restart(quiet=False):
    """Arm WITHOUT calibration or self-test, launch, stop 4 s into the glide, change the mission,
    arm again and fly to arrival. Exercises design section 12."""
    args = argparse.Namespace(fake=True, fin_ip="x", cam_ip="x", yolo=None, imu_axes="x,y,z",
                              guidance="laptop", host="127.0.0.1", port=0, open=False)
    st = GroundStation(args)
    st.start()
    assert st.command({"cmd": "connect"})["ok"]
    r = st.command({"cmd": "arm"}); assert not r["ok"], r          # no mission yet -> refused
    print("arm without mission ->", r["error"])
    assert st.command({"cmd": "mission", "speed": 0.8, "target": [2, 8, 1]})["ok"]
    r = st.command({"cmd": "arm"}); assert r["ok"] and r["skipped"], r
    print("arm without checks -> ok, skipped:", r["skipped"])
    assert st.command({"cmd": "arm"})["ok"] and st.phase == "armed"
    r = st.command({"cmd": "mission", "speed": 1.0, "target": [0, 6, 0]}); assert not r["ok"], r
    print("mission while armed ->", r["error"])
    assert st.command({"cmd": "launch"})["ok"]
    t0 = time.time()
    while time.time() - t0 < 10 and st.phase != "imu_glide":
        time.sleep(0.1)
    time.sleep(4.0)
    assert st.phase == "imu_glide", st.phase
    r = st.command({"cmd": "stop"}); time.sleep(0.3)
    print("stop mid-flight ->", r, "phase", st.phase, "fins", st.fin_cmd, "history rows kept", len(st.history))
    assert r["ok"] and st.phase == "ready" and st.fin_cmd == (0.0, 0.0) and len(st.history) > 10
    r = st.command({"cmd": "mission", "speed": 1.0, "target": [0, 6, 0]}); assert r["ok"], r
    print("re-applied mission ->", r)
    assert st.command({"cmd": "calibrate"})["ok"]                    # optional check still works in Ready
    time.sleep(st.cfg["calib_s"] + 0.4)
    assert st.command({"cmd": "arm"})["ok"] and st.command({"cmd": "arm"})["ok"] and st.phase == "armed"
    assert st.command({"cmd": "launch"})["ok"]
    fish = st.link
    t0 = time.time()
    while time.time() - t0 < 45 and st.phase not in ("arrived", "aborted"):
        time.sleep(0.5)
    with fish.lock:
        d = float(np.linalg.norm(fish.target - fish.pos))
    ok = st.phase == "arrived" and d < 0.6
    print("RESULT %s | stop-and-restart | second flight phase %s | true distance %.2f m" % ("PASS" if ok else "FAIL", st.phase, d))
    st.stop()
    return ok


def run(target, speed, abort_at=None, timeout=45.0, quiet=False, swarm=False):
    args = argparse.Namespace(fake=True, fin_ip="x", cam_ip="x", yolo=None, imu_axes="x,y,z",
                              guidance="laptop", host="127.0.0.1", port=0, open=False)
    st = GroundStation(args)
    st.start()
    for c in ({"cmd": "connect"}, {"cmd": "mission", "speed": speed, "target": target}, {"cmd": "calibrate"}):
        r = st.command(c)
        assert r["ok"], r
    if swarm:
        assert st.command({"cmd": "tune", "key": "vision_mode", "value": "swarm"})["ok"]
        assert st.command({"cmd": "tune", "key": "sim_quads", "value": 5})["ok"]
        r = st.command({"cmd": "swarm", "action": "next"}); print("swarm next before launch ->", r)
        r = st.command({"cmd": "swarm", "action": "auto"}); assert r["ok"], r
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
        if swarm and t["phase"] in ("cv_homing", "handover") and not getattr(run, "_swarm_seen", False) and t["cv"]["swarm_id"] is not None:
            run._swarm_seen = True
            print("swarm telemetry: mode=%s following S%s n=%d visible=%s quads(sim)=%d sources=%d" % (
                t["cv"]["mode"], t["cv"]["swarm_id"], t["cv"]["swarm_n"], [w["id"] for w in t["cv"]["swarms"]], len(t["sim"]["quads"]), t["cv"]["n_sources"]))
        if not quiet:
            print("%5.1f %-10s prog=%.2f true=(%.2f,%.2f,%.2f) est=(%.2f,%.2f,%.2f) err=%.2f dist=%.2f cv=%-9s conf=%.2f fins=(%.0f,%.0f)" % (
                time.time() - t0, t["phase"], t["progress"], *tp, e["x"], e["y"], e["z"], perr, d,
                t["cv"]["state"], t["cv"]["conf"], t["fins"]["yaw_cmd"], t["fins"]["pitch_cmd"]))
        if abort_at is not None and t["phase"] == "imu_glide" and time.time() - t0 >= abort_at:
            r = st.command({"cmd": "abort"})
            time.sleep(0.3)
            aborted_ok = r["ok"] and st.phase == "ready" and st.fin_cmd == (0.0, 0.0) and len(st.history) > 5
            r2 = st.command({"cmd": "arm"}); r2 = st.command({"cmd": "arm"}) if r2["ok"] else r2; armed = st.phase == "armed"; r3 = st.command({"cmd": "abort"})
            print("abort ->", r, "phase", st.phase, "fins", st.fin_cmd, "history", len(st.history), "| arm ->", r2["ok"], "| abort while armed ->", r3, "phase", st.phase)
            aborted_ok = aborted_ok and armed and r3["ok"] and st.phase == "ready"
            break
        if t["phase"] in ("arrived", "aborted"):
            break
        time.sleep(0.5)
    ok = aborted_ok if abort_at is not None else (st.phase == "arrived" and d < 0.9)
    if swarm:
        ok = ok and getattr(run, "_swarm_seen", False)
    print("RESULT %s | %starget %s at %.1f m/s | phase %s | true distance %.2f m | worst estimate error %.2f m | %.1f s" % (
        "PASS" if ok else "FAIL", "SWARM mode, " if swarm else "", target, speed, st.phase, d, worst, time.time() - t0))
    st.stop()
    return ok


if __name__ == "__main__":
    watchdog = threading.Timer(120, lambda: os._exit(3)); watchdog.daemon = True; watchdog.start()
    argv = [a for a in sys.argv[1:] if not a.startswith("--")]
    nums = [float(a) for a in argv]
    target = nums[:3] if len(nums) >= 3 else [2.0, 8.0, 1.0]
    speed = nums[3] if len(nums) >= 4 else 0.8
    if "--stop-restart" in sys.argv:
        ok = run_stop_restart(quiet="--quiet" in sys.argv)
    else:
        ok = run(target, speed, abort_at=4.0 if "--abort" in sys.argv else None, quiet="--quiet" in sys.argv, swarm="--swarm" in sys.argv)
    os._exit(0 if ok else 1)
