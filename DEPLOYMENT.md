# Drone Detector — Jetson Orin Nano Deployment Guide

**Audience:** engineers deploying the `small_drone` detector onboard.
**Goal:** run the model in **INT8** on an **8 GB Jetson Orin Nano** in real time.

---

## 1. Model card (what you're deploying)
- Architecture: **Ultralytics YOLO26-n**, single class `small_drone`, trained at **1280×1280**.
- v1 accuracy (held-out human-verified frames): **mAP@50 ≈ 0.76 · precision ≈ 0.82 · recall ≈ 0.74**.
- ONNX is **end-to-end / NMS-free** (output `(1, 300, 6)` = up to 300 dets of `[x1,y1,x2,y2,score,class]`). **No NMS step needed** — just a confidence threshold.
- ⚠️ **Known v1 limitations — set expectations:**
  - Recall falls to **~50% for far/small drones** (<~100 px² in frame). Strong up close, weak at distance.
  - Trained on **blue-LED drones in one indoor arena** → may not detect non-LED drones or other environments. v2 (more data) is in progress — see `DATA_CAPTURE.md`.

## 2. Artifacts (this `deploy/` folder)
| File | Purpose |
|---|---|
| `drone_n_v3.pt` | Ultralytics weights — **primary**, use for the engine export |
| `drone_n_v3_1280.onnx` | ONNX (opset 19, end2end) — for trtexec / non-Ultralytics runtimes |
| `calib_images/` | 400 representative frames for INT8 calibration |
| `calib.yaml` | dataset config for the Ultralytics INT8 export (edit `path`) |
| `scripts/build_engine.sh` | trtexec engine build (advanced path) |
| `scripts/benchmark.py` | latency / FPS / memory benchmark |
| `scripts/infer_image.py` | single-image sanity inference |

Copy the whole folder to the Jetson, e.g. `/home/jetson/drone_deploy/`, and set `path:` in `calib.yaml` to match.

## 3. Jetson prerequisites
- Jetson **Orin Nano (8 GB)**, **JetPack 6.x** (gives TensorRT 10.x + CUDA 12.x).
- Check: `cat /etc/nv_tegra_release` · `dpkg -l | grep -i tensorrt`
- **Max performance:** `sudo nvpmodel -m 0 && sudo jetson_clocks`
- Python deps: follow Ultralytics' **NVIDIA Jetson** guide for the correct torch/torchvision wheels, then `pip install ultralytics`. (Don't `pip install torch` from PyPI on Jetson — use NVIDIA's wheels.)

## 4. Build the INT8 engine — Path A (recommended)
> Engines are **device-specific** — build **on the Jetson**, never on a PC.
```bash
cd /home/jetson/drone_deploy
yolo export model=drone_n_v3.pt format=engine int8=True data=calib.yaml imgsz=1280 device=0 workspace=4
# → drone_n_v3.engine  (INT8, calibrated on calib_images/)
```
- `int8=True` + `data=` runs entropy calibration on `calib_images/`. **Calibration data quality is critical for small-object INT8** — keep these frames representative of real deployment.
- **Too slow / OOM at 1280?** Re-export at `imgsz=960` or `640` (recall drops with resolution — measure in §8).
- **FP16 fallback** (simpler, larger/slower, usually still real-time): `yolo export model=drone_n_v3.pt format=engine half=True imgsz=1280`.

## 5. Build the INT8 engine — Path B (advanced, trtexec)
Use `scripts/build_engine.sh` (ONNX → engine). It defaults to **FP16** because plain `trtexec --int8` without a calibrator gives poor accuracy. For calibrated INT8 prefer Path A. See script header.

## 6. Run + sanity check
```bash
yolo predict model=drone_n_v3.engine source=calib_images/run5__frame_000313.jpg imgsz=1280 conf=0.25
# or:  python scripts/infer_image.py drone_n_v3.engine calib_images/run5__frame_000313.jpg
```
Confirm boxes land on the drones (small blue-lit dots near the trusses).

## 7. Benchmark
```bash
python scripts/benchmark.py drone_n_v3.engine --imgsz 1280 --runs 300
# → mean/p90 latency, FPS
```
Target **≥ 30 FPS**. A nano model in INT8 at 1280 on the Orin Nano Super is expected to clear this comfortably — **measure and confirm**; if short, drop to 960/640.

## 8. Accuracy check (INT8 vs FP32)
Quantization can hurt small-object recall — verify it:
```bash
yolo val model=drone_n_v3.engine data=<your_val.yaml> imgsz=1280   # INT8
yolo val model=drone_n_v3.pt     data=<your_val.yaml> imgsz=1280   # FP32 reference
```
If INT8 mAP@50 drops by more than ~3–5 points (especially recall on small drones), go to §9.

## 9. If INT8 PTQ accuracy is unacceptable → QAT
Do PTQ (above) first. Only if the small-object drop is too large:
- Use **NVIDIA TensorRT Model Optimizer** (`modelopt`) or pytorch-quantization QAT: insert fake-quant nodes, **fine-tune a few epochs on the training set (on a PC/GPU)**, export a Q/DQ ONNX, then **build the engine on the Jetson**.
- Refs: NVIDIA "Achieving FP32 accuracy for INT8 inference using QAT", Ultralytics QAT notes.

## 10. Integration notes
- **Input:** letterbox to 1280×1280, **RGB**, normalized `/255`. Ultralytics handles this; if you write custom pre-processing for Path B, match it exactly (RGB order matters).
- **Post-process:** model is **NMS-free** → just threshold by score (start `conf=0.25`, tune for your recall/precision trade). No NMS/IoU step required via Path A.
- **Class:** single id `0 = small_drone`.
- **Power/thermal (flying platform):** INT8 + `nvpmodel` matters for the power budget; watch `tegrastats`.

## 11. Troubleshooting
| Symptom | Fix |
|---|---|
| Engine build OOM | lower `workspace`, free RAM, try `imgsz=640` |
| "serialized engine version mismatch" | rebuild on the **exact** JetPack/TensorRT of the device — engines aren't portable |
| Low FPS | `sudo jetson_clocks`, lower `imgsz`, confirm it's the INT8 (not FP32) engine |
| No detections | check `conf`, confirm RGB (not BGR) pre-proc, confirm `imgsz` matches export |

## 12. Camera change (D455 → Zed)
v1 was trained on **D455 RGB**. Switching to **Zed** is a domain shift (FOV/color pipeline) → budget a **short fine-tune on Zed frames**; keep the letterbox-to-1280 pre-processing fixed.

---
*Questions on training/data: see `DATA_CAPTURE.md` and the pipeline scripts in `~/drone_work/scripts/`.*
