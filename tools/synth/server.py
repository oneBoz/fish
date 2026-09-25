"""
Save server for the synthetic drone generator (tools/synth/index.html).

    python tools/synth/server.py                 # dataset in datasets/quads, page at http://127.0.0.1:9100
    python tools/synth/server.py --out /data/quads --port 9100

Serves the generator page and accepts POST /save {name, split, jpeg(base64), label} from it,
writing images/<split>/<name>.jpg and labels/<split>/<name>.txt in YOLO format, plus data.yaml.
Standard library only.
"""
import argparse
import base64
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, "..", "..", "datasets", "quads"))


def write_yaml(out):
    with open(os.path.join(out, "data.yaml"), "w") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n  0: quad\n" % out)


def counts(out):
    def n(split):
        d = os.path.join(out, "images", split)
        return len([f for f in os.listdir(d) if f.endswith(".jpg")]) if os.path.isdir(d) else 0
    return {"train": n("train"), "val": n("val")}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # quiet
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/stats":
            self._json(200, {"out": OUT, **counts(OUT)})
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/save":
            return self._json(404, {"error": "not found"})
        try:
            n = int(self.headers.get("Content-Length", "0"))
            d = json.loads(self.rfile.read(n))
            name = "".join(c for c in str(d["name"]) if c.isalnum() or c in "_-")
            split = "val" if d.get("split") == "val" else "train"
            if not name:
                raise ValueError("bad name")
            img_dir = os.path.join(OUT, "images", split)
            lbl_dir = os.path.join(OUT, "labels", split)
            os.makedirs(img_dir, exist_ok=True)
            os.makedirs(lbl_dir, exist_ok=True)
            with open(os.path.join(img_dir, name + ".jpg"), "wb") as f:
                f.write(base64.b64decode(d["jpeg"]))
            with open(os.path.join(lbl_dir, name + ".txt"), "w") as f:
                f.write((d.get("label") or "").strip() + ("\n" if d.get("label") else ""))
            self._json(200, {"ok": True})
        except Exception as e:  # noqa: BLE001
            self._json(400, {"ok": False, "error": str(e)})


def main():
    global OUT
    p = argparse.ArgumentParser(description="Save server for the synthetic drone generator")
    p.add_argument("--out", default=OUT, help="dataset folder (default datasets/quads)")
    p.add_argument("--port", type=int, default=9100)
    p.add_argument("--host", default="127.0.0.1")
    a = p.parse_args()
    OUT = os.path.abspath(a.out)
    os.makedirs(OUT, exist_ok=True)
    write_yaml(OUT)
    print("Quad Synth at http://%s:%d  ->  %s" % (a.host, a.port, OUT), flush=True)
    ThreadingHTTPServer((a.host, a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
