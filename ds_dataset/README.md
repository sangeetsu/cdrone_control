# DS Dataset Workspace

This folder is the capture workspace for collecting a better small-drone RGB
dataset from the RealSense D455 while the main drone is being flown manually.

The default capture profile is intentionally simple for YOLO data collection:

- color `1280x800@15`
- RGB frames only by default
- optional depth capture only if we explicitly turn it on later
- saved image cadence `2 FPS` which is one image every `0.5` seconds
- raw session capture first, YOLO annotation later

## Layout

```text
ds_dataset/
  sessions/                  raw capture sessions land here
  exports/yolo/              later YOLO-ready train/val/test export
  runtime/                   pid/log files for active capture sessions
  scripts/
    record_realsense_dataset.py
    start_capture.sh
    stop_capture.sh
    status_capture.sh
    export_yolo_dataset.py
```

Each session is written as:

```text
sessions/<session_name>/
  rgb/                       color frames as .jpg
  depth/                     optional aligned depth frames as 16-bit .png
  annotations_yolo/          put YOLO .txt labels here later
  manifest.json              capture settings and camera metadata
  frames.csv                 per-frame metadata and expected label path
  annotation_index.csv       pending-annotation scaffold
```

## Manual Flight Workflow

1. Get the RealSense connected and the drone ready for manual flight.
2. Start flying.
3. Tell me to start recording, or run:

```bash
./ds_dataset/scripts/start_capture.sh small_drone_flight
```

4. Let the session run while you fly and while the smaller target drones move
   through frame.
5. Tell me to stop recording, or run:

```bash
./ds_dataset/scripts/stop_capture.sh
```

6. Check the active session:

```bash
./ds_dataset/scripts/status_capture.sh
```

The recorder writes raw capture data only. It does not create fake empty YOLO
labels during collection.

If you ever want depth for later analysis, start the recorder with `--save-depth`.

`start_capture.sh` launches the recorder as a detached background process, so it
keeps recording until `stop_capture.sh` is run or the recorder hits an explicit
limit like `--max-frames` or `--max-seconds`.

Live runtime state lands in `ds_dataset/runtime/`, including a per-session log
and status JSON file. `status_capture.sh` is the best way to check whether a
session is still running and how many frames have been saved so far.

## Annotation Convention

For each captured image:

- expected label path:
  `sessions/<session>/annotations_yolo/<frame_stem>.txt`
- missing `.txt` file means "not annotated yet"
- existing empty `.txt` file means "annotated and intentionally negative"
- existing non-empty `.txt` file means YOLO labels are present

The initial class scaffold is:

- class `0`: `small_drone`

If you later want more classes, update `exports/yolo/data.yaml` before training.

## Export Later For YOLO Training

Once labels exist, build the train/val/test export:

```bash
python3 ds_dataset/scripts/export_yolo_dataset.py \
  --session 20260417_120000_small_drone_flight
```

That script copies or links only frames that already have matching YOLO label
files and refreshes:

- `ds_dataset/exports/yolo/images/train|val|test`
- `ds_dataset/exports/yolo/labels/train|val|test`
- `ds_dataset/exports/yolo/data.yaml`
- `ds_dataset/exports/yolo/manifests/export_manifest.csv`

## Notes

- The recorder is intentionally separate from the active tracker so we can
  collect RGB training data without disturbing the current `drone_vision_pkg`
  edits.
- Session names are timestamped so each flight produces an isolated capture set.
- If depth is enabled, depth frames are saved as raw aligned `z16` PNG plus
  `depth_scale_m` in the manifest so the data stays reusable for later analysis.
