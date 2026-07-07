# Data Capture Spec — Drone Detector v2

**Why this exists:** v1 recall is capped at **~0.74**, and the failure analysis is unambiguous — **far/small drones are missed ~50% of the time**, and there are only **6 such examples in the entire 4,900-frame v1 set**. No model/architecture change moved this (we tried standard, P2-head, and LRDD-pretrained — recall stayed ~0.76 across all). **It's a data gap, not a modeling problem.** This spec says exactly what to capture to fix it.

> Guiding principle: **diversity beats volume.** 500 well-chosen new frames will help more than 5,000 more of the same arena footage.

## Capture priorities (in order)

### 1. FAR / SMALL drones — top priority
- Fly drones to the **far end of the arena and beyond**; target an in-frame size of **< ~100 px** (a few-pixel speck in a 1280-wide image).
- Sweep distance continuously near → far so every scale is represented.
- **Target: ≥ 2,000 frames** with drones in the far/small regime.

### 2. Different / non-LED drones
- v1 likely keys on the **blue-LED glow**. Capture with **LEDs off**, different LED colors, and **different airframes/sizes** (other quads; fixed-wing if relevant).
- **Target: ≥ 1,000 frames**, **≥ 2–3 distinct drones**, a good fraction with LEDs **off**.

### 3. Varied environment / lighting
- Different rooms, **outdoor**, different times of day, window light. Breaks overfit to this one arena.
- **Target: ≥ 1,000 frames** across **≥ 2 new settings**.

### 4. Hard negatives (kills false-positives)
- Frames with the **blue monitors/equipment** that triggered v1 false-positives, plus other blue/bright clutter — **with no drone present**.
- **Target: ≥ 500 frames**, drone-free.

## Capture settings — changes from v1 (important)
- ✅ **Enable depth** (`save_depth: true`). v1 had it off; depth makes auto-annotation far easier and enables future RGBD work.
- ✅ **Higher save FPS** (≈ 5–10, up from 2). v1's 2 fps was too sparse for tracker/auto-label propagation.
- Log **distance + conditions per clip** (a CSV like the LRDD dataset's is ideal — drone range, lighting, background).
- Keep RealSense intrinsics/exposure consistent within a session.
- If moving to **Zed**, capture a parallel set for the fine-tune.

## Feeding new data back into the pipeline
1. Drop each session under `~/drone_work/data/raw/<new_run>/rgb/`.
2. **Pre-label with the trained model** (NOT the blue-LED color proposer — it won't generalize to non-LED drones):
   ```bash
   yolo predict model=~/yolo_runs/n_v3/weights/best.pt source=<new_run>/rgb imgsz=1280 conf=0.2 save_txt
   ```
3. **Verify/correct in CVAT** (same loop as before; CVAT task creation via `scripts/` + cvat-cli). Spend effort on the **far/small + non-LED** frames — that's where the model is weak.
4. **Retrain** `yolo26-p2` (or `n`) on the expanded set. With real far/small examples present, the recall ceiling should finally move.

## What NOT to collect
- More **near/medium** drones in **this same arena** — we already have plenty (recall there is ~100%). Adding more won't help.

## Success criteria for v2
- Recall on the **<100 px** bucket up from ~50% → **> 80%**.
- Model fires on a **non-LED** drone it has never seen.
- False-positives on blue-equipment clutter near zero.
