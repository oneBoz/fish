"""FastAPI server: static GCS page, /ws telemetry, /cmd, /video_feed, /log.json, /log.csv."""
import argparse
import asyncio
import json
import os
import webbrowser
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse, Response, StreamingResponse

from .station import GroundStation

STATIC = os.path.join(os.path.dirname(__file__), "static")
station: GroundStation = None  # set in main()


@asynccontextmanager
async def lifespan(app):
    station.start()
    yield
    station.stop()


app = FastAPI(title="Fish GCS", lifespan=lifespan)


@app.get("/")
def index():
    return FileResponse(os.path.join(STATIC, "index.html"), headers={"Cache-Control": "no-store"})


@app.post("/cmd")
async def cmd(req: Request):
    try:
        body = await req.json()
    except Exception:  # noqa: BLE001
        return JSONResponse({"ok": False, "error": "Body must be JSON."}, status_code=400)
    return JSONResponse(await asyncio.to_thread(station.command, body))


@app.get("/telemetry")
def telemetry():
    return JSONResponse(station.telemetry)


@app.get("/events")
def events(since: int = 0):
    return JSONResponse([e for e in station.events if e["seq"] > since])


@app.get("/log.json")
def log_json():
    return Response(station.export_json(), media_type="application/json", headers={"Content-Disposition": "attachment; filename=fish-flight.json"})


@app.get("/log.csv")
def log_csv():
    return PlainTextResponse(station.export_csv(), headers={"Content-Disposition": "attachment; filename=fish-flight.csv"})


@app.get("/video_feed")
async def video_feed():
    async def gen():
        last = None
        while True:
            jpeg = station.vision.jpeg if station.vision else None
            if jpeg is not None and jpeg is not last:
                last = jpeg
                yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n"
            await asyncio.sleep(0.05)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame", headers={"Cache-Control": "no-store"})


@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    seq = 0
    events = list(station.events)
    if events:
        seq = events[-1]["seq"]
    await sock.send_text(json.dumps({"type": "hello", "events": events[-80:], "fake": bool(station.args.fake), "defaults": {"fin_ip": station.args.fin_ip, "cam_ip": station.args.cam_ip}}))
    try:
        while True:
            new = [e for e in station.events if e["seq"] > seq]
            for e in new:
                seq = e["seq"]
                await sock.send_text(json.dumps({"type": "event", **e}))
            if station.telemetry:
                await sock.send_text(json.dumps(station.telemetry))
            await asyncio.sleep(0.05)
    except (WebSocketDisconnect, RuntimeError):
        pass


def parse_args(argv=None):
    p = argparse.ArgumentParser(prog="python -m gcs", description="Fish ground control station")
    p.add_argument("--fake", action="store_true", help="no hardware: simulate the fish, its IMU and its IR camera")
    p.add_argument("--fin-ip", default="172.20.10.12", help="fish node (ESP32-S3) IP, UDP 4210")
    p.add_argument("--cam-ip", default="172.20.10.13", help="ESP32-CAM IP (MJPEG on :81/stream)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=9000)
    p.add_argument("--yolo", default=None, help="YOLO nano weights (.pt); needs `pip install ultralytics`")
    p.add_argument("--imu-axes", default="x,y,z", help="ICM20948 sensor axes that point forward, right, up (default x,y,z; must stay right-handed, e.g. x,-y,-z)")
    p.add_argument("--guidance", choices=["laptop", "fish"], default="laptop", help="who steers during IMU glide")
    p.add_argument("--open", action="store_true", help="open the GCS in the default browser")
    return p.parse_args(argv)


def main(argv=None):
    global station
    args = parse_args(argv)
    station = GroundStation(args)
    url = "http://%s:%d" % (args.host, args.port)
    print("Fish GCS at", url, "(fake mode)" if args.fake else "", flush=True)
    if args.open:
        webbrowser.open(url)
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
