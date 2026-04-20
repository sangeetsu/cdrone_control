#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import os
import shutil
from pathlib import Path


FRAME_CSV_NAME = "frames.csv"
EXPORT_MANIFEST_NAME = "export_manifest.csv"


def default_dataset_root() -> Path:
    return Path(__file__).resolve().parents[1]


def stable_split_key(seed: int, key: str) -> float:
    digest = hashlib.sha256(f"{seed}:{key}".encode("utf-8")).hexdigest()
    return int(digest[:12], 16) / float(16**12)


def choose_split(score: float, val_fraction: float, test_fraction: float) -> str:
    if score < test_fraction:
        return "test"
    if score < test_fraction + val_fraction:
        return "val"
    return "train"


def ensure_output_dirs(output_root: Path) -> None:
    for relative in (
        Path("images/train"),
        Path("images/val"),
        Path("images/test"),
        Path("labels/train"),
        Path("labels/val"),
        Path("labels/test"),
        Path("manifests"),
    ):
        (output_root / relative).mkdir(parents=True, exist_ok=True)


def resolve_sessions(dataset_root: Path, requested_sessions: list[str]) -> list[Path]:
    sessions_root = dataset_root / "sessions"
    if requested_sessions:
        return [sessions_root / name for name in requested_sessions]
    return sorted(path for path in sessions_root.iterdir() if path.is_dir())


def transfer_file(src: Path, dst: Path, mode: str) -> None:
    if dst.exists() or dst.is_symlink():
        dst.unlink()
    if mode == "copy":
        shutil.copy2(src, dst)
    elif mode == "symlink":
        dst.symlink_to(src)
    elif mode == "hardlink":
        os.link(src, dst)
    else:
        raise ValueError(f"Unsupported transfer mode: {mode}")


def write_data_yaml(output_root: Path, class_name: str) -> None:
    yaml_path = output_root / "data.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                f"path: {output_root}",
                "train: images/train",
                "val: images/val",
                "test: images/test",
                "names:",
                f"  0: {class_name}",
                "",
            ]
        ),
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Export annotated raw capture sessions into a YOLO-ready train/val/test "
            "folder structure."
        )
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=default_dataset_root(),
        help="Root of the ds_dataset workspace.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=default_dataset_root() / "exports" / "yolo",
        help="Destination YOLO dataset root.",
    )
    parser.add_argument(
        "--session",
        action="append",
        default=[],
        help="Specific session folder name to export. Repeat to include multiple.",
    )
    parser.add_argument(
        "--val-fraction",
        type=float,
        default=0.15,
        help="Fraction of labeled samples to route into val.",
    )
    parser.add_argument(
        "--test-fraction",
        type=float,
        default=0.05,
        help="Fraction of labeled samples to route into test.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Stable split seed.",
    )
    parser.add_argument(
        "--class-name",
        default="small_drone",
        help="Class name to write into data.yaml for class index 0.",
    )
    parser.add_argument(
        "--mode",
        choices=("copy", "symlink", "hardlink"),
        default="copy",
        help="How to place files into the export tree.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be exported without writing files.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    dataset_root = args.dataset_root.resolve()
    output_root = args.output_root.resolve()
    sessions = resolve_sessions(dataset_root, args.session)

    if not sessions:
        print("No session folders found.")
        return 0

    if args.val_fraction < 0.0 or args.test_fraction < 0.0:
        raise SystemExit("Split fractions must be non-negative.")
    if args.val_fraction + args.test_fraction >= 1.0:
        raise SystemExit("val_fraction + test_fraction must stay below 1.0.")

    if not args.dry_run:
        ensure_output_dirs(output_root)

    manifest_rows: list[dict[str, str]] = []
    exported_count = 0

    for session_dir in sessions:
        frames_csv_path = session_dir / FRAME_CSV_NAME
        if not frames_csv_path.exists():
            print(f"Skipping {session_dir.name}: missing {FRAME_CSV_NAME}")
            continue

        with frames_csv_path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rgb_path = session_dir / row["rgb_relpath"]
                label_path = session_dir / row["yolo_label_relpath"]
                if not rgb_path.exists():
                    continue
                if not label_path.exists():
                    continue

                source_key = f"{session_dir.name}/{row['frame_stem']}"
                split = choose_split(
                    stable_split_key(args.seed, source_key),
                    val_fraction=args.val_fraction,
                    test_fraction=args.test_fraction,
                )
                image_ext = rgb_path.suffix
                label_ext = label_path.suffix or ".txt"
                export_stem = f"{session_dir.name}__{row['frame_stem']}"
                dst_image = output_root / "images" / split / f"{export_stem}{image_ext}"
                dst_label = output_root / "labels" / split / f"{export_stem}{label_ext}"

                if not args.dry_run:
                    transfer_file(rgb_path, dst_image, args.mode)
                    transfer_file(label_path, dst_label, args.mode)

                manifest_rows.append(
                    {
                        "session_name": session_dir.name,
                        "frame_stem": row["frame_stem"],
                        "split": split,
                        "source_image": str(rgb_path),
                        "source_label": str(label_path),
                        "dest_image": str(dst_image),
                        "dest_label": str(dst_label),
                    }
                )
                exported_count += 1

    if args.dry_run:
        print(f"Dry run would export {exported_count} labeled frames.")
        return 0

    write_data_yaml(output_root, args.class_name)
    manifest_path = output_root / "manifests" / EXPORT_MANIFEST_NAME
    with manifest_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "session_name",
                "frame_stem",
                "split",
                "source_image",
                "source_label",
                "dest_image",
                "dest_label",
            ],
        )
        writer.writeheader()
        writer.writerows(manifest_rows)

    print(f"Exported {exported_count} labeled frames into {output_root}")
    print(f"Manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
