#!/usr/bin/env python3
"""Summarize a recorded Quest controller tracking session."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_samples(path: Path) -> list[dict]:
    with path.open() as f:
        return [json.loads(line) for line in f if line.strip()]


def axis_ranges(samples: list[dict]) -> tuple[list[float], list[float], list[float]]:
    positions = [sample["position"] for sample in samples if sample.get("position")]
    if not positions:
        return [], [], []
    mins = [min(pos[i] for pos in positions) for i in range(3)]
    maxs = [max(pos[i] for pos in positions) for i in range(3)]
    spans = [maxs[i] - mins[i] for i in range(3)]
    return mins, maxs, spans


def summarize_segments(samples: list[dict]) -> list[dict]:
    segments: dict[int, dict] = {}
    for sample in samples:
        sid = int(sample.get("segment_id", 0))
        if sid <= 0:
            continue
        stats = segments.setdefault(
            sid,
            {
                "segment_id": sid,
                "start_time": sample["time"],
                "end_time": sample["time"],
                "samples": 0,
                "path_m": 0.0,
            },
        )
        stats["end_time"] = sample["time"]
        stats["samples"] += 1
        stats["path_m"] += float(sample.get("delta_m", 0.0))
    return list(segments.values())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("samples", type=Path, help="Path to quest_tracking_*/samples.jsonl")
    args = parser.parse_args()

    samples = load_samples(args.samples)
    mins, maxs, spans = axis_ranges(samples)
    segments = summarize_segments(samples)
    moving = sum(1 for sample in samples if sample.get("moving"))
    duration = samples[-1]["time"] - samples[0]["time"] if len(samples) >= 2 else 0.0
    path_m = sum(float(sample.get("delta_m", 0.0)) for sample in samples)

    print(f"samples: {len(samples)}")
    print(f"duration_s: {duration:.3f}")
    print(f"moving_samples: {moving}")
    print(f"total_path_m: {path_m:.3f}")
    if mins:
        print(f"min_xyz_m: {[round(v, 4) for v in mins]}")
        print(f"max_xyz_m: {[round(v, 4) for v in maxs]}")
        print(f"span_xyz_m: {[round(v, 4) for v in spans]}")
    print(f"motion_segments: {len(segments)}")
    for segment in segments[:20]:
        seg_duration = segment["end_time"] - segment["start_time"]
        print(
            f"  segment {segment['segment_id']}: "
            f"samples={segment['samples']} duration_s={seg_duration:.3f} path_m={segment['path_m']:.3f}"
        )


if __name__ == "__main__":
    main()
