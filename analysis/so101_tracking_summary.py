#!/usr/bin/env python3
"""Compare leader commands with measured follower positions in a LeRobot dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


def load_recording(root: Path) -> tuple[np.ndarray, np.ndarray, list[str], float]:
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"LeRobot metadata not found: {info_path}")

    info = json.loads(info_path.read_text())
    names = info["features"]["action"].get("names")
    if not names:
        raise ValueError("The dataset does not contain action joint names")

    parquet_files = sorted((root / "data").glob("**/*.parquet"))
    if not parquet_files:
        raise FileNotFoundError(f"No Parquet data found below {root / 'data'}")

    tables = [pq.read_table(path, columns=["action", "observation.state"]) for path in parquet_files]
    table = pa.concat_tables(tables)
    leader = np.asarray(table["action"].combine_chunks().to_pylist(), dtype=np.float64)
    follower = np.asarray(table["observation.state"].combine_chunks().to_pylist(), dtype=np.float64)
    if leader.shape != follower.shape or leader.ndim != 2:
        raise ValueError(f"Unexpected action/state shapes: {leader.shape} and {follower.shape}")

    return leader, follower, list(names), float(info["fps"])


def estimate_lag_frames(leader: np.ndarray, follower: np.ndarray, max_frames: int) -> int | None:
    if np.ptp(leader) < 1.0 or np.ptp(follower) < 1.0:
        return None

    best_lag = 0
    best_error = float("inf")
    for lag in range(max_frames + 1):
        lead = leader[: len(leader) - lag] if lag else leader
        follow = follower[lag:] if lag else follower
        if len(lead) < 10:
            continue
        # Remove a constant calibration offset before comparing trajectory shape.
        residual = (follow - np.mean(follow)) - (lead - np.mean(lead))
        error = float(np.sqrt(np.mean(residual**2)))
        if error < best_error:
            best_error = error
            best_lag = lag
    return best_lag


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Summarize SO-101 leader/follower tracking from a lerobot-record dataset."
    )
    parser.add_argument("dataset", type=Path, help="Local root passed to --dataset.root")
    parser.add_argument("--max-lag-ms", type=float, default=500.0)
    args = parser.parse_args()

    leader, follower, names, fps = load_recording(args.dataset.expanduser().resolve())
    max_lag_frames = min(len(leader) // 3, max(0, round(args.max_lag_ms * fps / 1000.0)))

    print(f"samples={len(leader)} fps={fps:g} duration_s={len(leader) / fps:.2f}")
    print("error = measured follower - leader command")
    print(
        f"{'joint':<16} {'leader range':>23} {'follower range':>23} "
        f"{'mean err':>10} {'MAE':>8} {'RMSE':>8} {'max':>8} {'lag':>9}"
    )
    for index, name in enumerate(names):
        lead = leader[:, index]
        follow = follower[:, index]
        error = follow - lead
        lag = estimate_lag_frames(lead, follow, max_lag_frames)
        lag_text = "n/a" if lag is None else f"{1000.0 * lag / fps:.0f} ms"
        unit = "%" if name.startswith("gripper") else "deg"
        leader_range = f"{lead.min():7.2f}..{lead.max():7.2f}"
        follower_range = f"{follow.min():7.2f}..{follow.max():7.2f}"
        print(
            f"{name:<16} {leader_range:>19} {unit:<3} {follower_range:>19} {unit:<3} "
            f"{error.mean():10.2f} {np.mean(np.abs(error)):8.2f} "
            f"{np.sqrt(np.mean(error**2)):8.2f} {np.max(np.abs(error)):8.2f} {lag_text:>9}"
        )


if __name__ == "__main__":
    main()
