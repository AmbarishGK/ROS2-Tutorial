#!/usr/bin/env python3
"""
Simple YOLOv8 inference with OpenCV display.
- Supports webcam (source=0), video file, or single image.
- Saves annotated output if --save is provided.
- Uses CUDA if available, otherwise CPU.
"""

import argparse
import os
import time
from pathlib import Path

import cv2
import torch
import numpy as np
from ultralytics import YOLO


def draw_dets(frame, result):
    """Draw bounding boxes + labels on a frame from a YOLO result."""
    if not hasattr(result, "boxes") or result.boxes is None:
        return frame, 0
    names = result.names if hasattr(result, "names") else {}
    det_count = 0
    for b in result.boxes:
        xyxy = b.xyxy[0].tolist()
        x1, y1, x2, y2 = map(int, xyxy)
        conf = float(b.conf[0].item()) if b.conf is not None else 0.0
        cls_id = int(b.cls[0].item()) if b.cls is not None else -1
        cls_name = names.get(cls_id, str(cls_id))
        det_count += 1

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)
        label = f"{cls_name} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        y_text = max(0, y1 - 6)
        cv2.rectangle(frame, (x1, y_text - th - 4), (x1 + tw + 2, y_text), (0, 200, 0), -1)
        cv2.putText(frame, label, (x1 + 1, y_text - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    return frame, det_count


def infer_image(model, device, src_path, save_path=None, conf=0.25):
    img = cv2.imread(str(src_path))
    if img is None:
        raise FileNotFoundError(f"Could not read image: {src_path}")
    t0 = time.time()
    results = model(img, conf=conf, verbose=False, device=device)
    dt_ms = (time.time() - t0) * 1000.0
    annotated, n = draw_dets(img.copy(), results[0])
    print(f"[image] {src_path} -> {n} dets in {dt_ms:.1f} ms (device={device})")
    if save_path:
        cv2.imwrite(str(save_path), annotated)
        print(f"Saved: {save_path}")
    cv2.imshow("YOLOv8 - Image", annotated)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def infer_video(model, device, src, save_path=None, conf=0.25, width=None, height=None):
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open source: {src}")

    if width:  cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    if height: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

    writer = None
    if save_path:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        writer = cv2.VideoWriter(str(save_path), fourcc, fps, (w, h))
        print(f"Recording to: {save_path} ({w}x{h}@{fps:.1f})")

    win = "YOLOv8 - Video"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t0 = time.time()
        results = model(frame, conf=conf, verbose=False, device=device)
        annotated, n = draw_dets(frame.copy(), results[0])
        dt_ms = (time.time() - t0) * 1000.0
        cv2.putText(annotated, f"{n} dets | {dt_ms:.1f} ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 200, 20), 2)
        cv2.imshow(win, annotated)
        if writer:
            writer.write(annotated)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):  # ESC or q
            break

    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="YOLOv8 inference with OpenCV display")
    ap.add_argument("--model", default="yolov8n.pt", help="Path or name of YOLOv8 model")
    ap.add_argument("--source", default="0",
                    help="0 (webcam), path to image/video, or RTSP/URL")
    ap.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    ap.add_argument("--save", action="store_true", help="Save annotated output")
    ap.add_argument("--width", type=int, default=None, help="Force capture width")
    ap.add_argument("--height", type=int, default=None, help="Force capture height")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = YOLO(args.model)
    # NOTE: No .to(device) call; Ultralytics handles device selection via `device` argument.

    src = args.source
    save_path = None
    if args.save:
        out_dir = Path("yolo_out")
        out_dir.mkdir(parents=True, exist_ok=True)
        stem = "webcam" if src == "0" else Path(src).stem
        save_path = out_dir / f"{stem}_annotated.mp4"  # will switch to .jpg for images below

    # Decide if image, video/webcam, or URL
    if src == "0":
        # webcam
        infer_video(model, device, 0, save_path=save_path, conf=args.conf,
                    width=args.width, height=args.height)
    else:
        p = Path(src)
        if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"} and p.exists():
            if args.save:
                save_path = Path("yolo_out") / f"{p.stem}_annotated.jpg"
            infer_image(model, device, p, save_path=save_path, conf=args.conf)
        else:
            # treat as video file or URL/RTSP
            infer_video(model, device, src, save_path=save_path, conf=args.conf,
                        width=args.width, height=args.height)


if __name__ == "__main__":
    main()
