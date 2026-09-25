"""
Fine-tune YOLO nano on the synthetic quad dataset and export ONNX for the Raspberry Pi.

    .venv/bin/pip install ultralytics                     # once (pulls PyTorch)
    .venv/bin/python tools/train_quads.py                 # datasets/quads/data.yaml, 50 epochs, 320 px
    .venv/bin/python tools/train_quads.py --epochs 80 --imgsz 640 --model yolo11n.pt
    .venv/bin/python tools/train_quads.py --check         # run the exported model on a few val images

Outputs models/quads.pt and models/quads.onnx, then prints the commands to use them:
    python -m gcs --yolo models/quads.pt --fake              (laptop)
    python -m gcs.vision --stream http://<cam>:81/stream --yolo models/quads.onnx --mode swarm   (Pi)
"""
import argparse
import glob
import os
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DATA = os.path.join(ROOT, "datasets", "quads", "data.yaml")
MODELS = os.path.join(ROOT, "models")


def need_ultralytics():
    try:
        from ultralytics import YOLO  # noqa: F401
        return True
    except ImportError:
        print("ultralytics is not installed. Run:  %s -m pip install ultralytics" % sys.executable)
        return False


def count(split):
    d = os.path.join(os.path.dirname(DATA), "images", split)
    return len(glob.glob(os.path.join(d, "*.jpg")))


def train(a):
    from ultralytics import YOLO
    n_train, n_val = count("train"), count("val")
    if n_train < 50 or n_val < 5:
        print("Dataset too small (%d train, %d val). Generate images first: python tools/synth/server.py and open http://127.0.0.1:9100" % (n_train, n_val))
        return 1
    print("Training on %d train / %d val images, %d epochs at %d px, base %s" % (n_train, n_val, a.epochs, a.imgsz, a.model))
    model = YOLO(a.model)
    model.train(data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, project=os.path.join(ROOT, "runs"), name="quads", exist_ok=True,
                device=a.device, workers=2, plots=True, verbose=False,
                hsv_h=0.02, hsv_s=0.5, hsv_v=0.5, degrees=10, scale=0.4, fliplr=0.5, mosaic=1.0)
    best = os.path.join(ROOT, "runs", "quads", "weights", "best.pt")
    os.makedirs(MODELS, exist_ok=True)
    shutil.copy(best, os.path.join(MODELS, "quads.pt"))
    onnx = YOLO(best).export(format="onnx", imgsz=a.export_imgsz, opset=12, simplify=True)
    shutil.copy(onnx, os.path.join(MODELS, "quads.onnx"))
    with open(os.path.join(MODELS, "quads.names"), "w") as f:
        f.write("quad\n")
    metrics = YOLO(best).val(data=a.data, imgsz=a.imgsz, verbose=False)
    print("mAP50 %.3f  mAP50-95 %.3f" % (metrics.box.map50, metrics.box.map))
    print("Saved models/quads.pt and models/quads.onnx")
    print("Laptop:  python -m gcs --fake --yolo models/quads.pt")
    print("Pi:      python -m gcs.vision --stream http://<cam-ip>:81/stream --yolo models/quads.onnx --mode swarm")
    return 0


def check(a):
    sys.path.insert(0, ROOT)
    from gcs.yolo_backend import YoloBackend
    import cv2
    w = os.path.join(MODELS, "quads.onnx") if os.path.exists(os.path.join(MODELS, "quads.onnx")) else os.path.join(MODELS, "quads.pt")
    if not os.path.exists(w):
        print("No trained model in models/. Train first.")
        return 1
    be = YoloBackend(w, imgsz=a.export_imgsz, conf=0.25)
    imgs = sorted(glob.glob(os.path.join(os.path.dirname(DATA), "images", "val", "*.jpg")))[:8]
    for p in imgs:
        boxes = be.detect(cv2.imread(p))
        truth = os.path.join(os.path.dirname(DATA), "labels", "val", os.path.basename(p)[:-4] + ".txt")
        n_true = sum(1 for line in open(truth) if line.strip()) if os.path.exists(truth) else 0
        print("%-40s %d detected / %d labelled, best conf %.2f, %.0f ms" % (os.path.basename(p), len(boxes), n_true, boxes[0][4] if boxes else 0.0, be.last_ms))
    return 0


def main():
    p = argparse.ArgumentParser(description="Train YOLO nano on synthetic quads and export ONNX")
    p.add_argument("--data", default=DATA)
    p.add_argument("--model", default="yolo11n.pt", help="base weights (downloaded by ultralytics on first use)")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--imgsz", type=int, default=320)
    p.add_argument("--export-imgsz", type=int, default=320, help="ONNX input size for the Pi")
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--device", default=None, help="cpu, 0, mps ... (default: auto)")
    p.add_argument("--check", action="store_true", help="run the exported model on a few val images")
    a = p.parse_args()
    if a.check:
        sys.exit(check(a))
    if not need_ultralytics():
        sys.exit(1)
    sys.exit(train(a))


if __name__ == "__main__":
    main()
