#!/usr/bin/env python3
"""Measure live occupancy-map fusion and publication rates from a run directory."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

NS_PER_SECOND = 1_000_000_000
EVENT_TIME = "completed_monotonic_ns"


def read_summary(path: Path) -> dict[str, str]:
    # These top-level scalar fields are written by OpenCV FileStorage.
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(": ")
        if separator:
            fields[key] = value.strip().strip('"')
    return fields


def count_updates(path: Path, start_ns: int, end_ns: int) -> tuple[int, int, int, int, list[int]]:
    fused = published = total_fused = total_published = 0
    per_second = [0] * ((end_ns - start_ns) // NS_PER_SECOND)
    previous_ns = None
    with path.open(newline="", encoding="utf-8") as stream:
        rows = csv.DictReader(stream)
        required = {"accepted", "published", EVENT_TIME}
        missing = required.difference(rows.fieldnames or ())
        if missing:
            raise ValueError(f"{path.name} is missing columns: {', '.join(sorted(missing))}")

        for line, row in enumerate(rows, start=2):
            stamp_ns = int(row[EVENT_TIME])
            accepted, delivered = row["accepted"], row["published"]
            if (accepted, delivered) not in {("0", "0"), ("1", "0"), ("1", "1")}:
                raise ValueError(f"{path.name}:{line}: invalid fusion/publication flags")
            if previous_ns is not None and stamp_ns < previous_ns:
                raise ValueError(f"{path.name}:{line}: event times are out of order")
            previous_ns = stamp_ns
            total_fused += accepted == "1"
            total_published += delivered == "1"

            # Count the live window only. Work finishing during drain stays in the
            # whole-run totals so those still agree with summary.yml.
            if not start_ns <= stamp_ns < end_ns:
                continue
            fused += accepted == "1"
            if delivered == "1":
                published += 1
                second = (stamp_ns - start_ns) // NS_PER_SECOND
                if second < len(per_second):
                    per_second[second] += 1

    return fused, published, total_fused, total_published, per_second


def measure(run: Path, seconds: int) -> dict:
    summary = read_summary(run / "summary.yml")
    if summary.get("mode") != "live":
        raise ValueError("summary.yml must describe a live mapping run")
    start_ns = int(summary["window_start_monotonic_ns"])
    input_end_ns = int(summary["window_end_monotonic_ns"])
    if input_end_ns <= start_ns:
        raise ValueError("summary.yml has an invalid observation window")
    requested_end_ns = start_ns + seconds * NS_PER_SECOND
    end_ns = min(input_end_ns, requested_end_ns)

    fused, published, total_fused, total_published, per_second = count_updates(
        run / "mapping.csv", start_ns, end_ns
    )
    expected_fused = float(summary["fused"])
    expected_skipped = float(summary["snapshot_publish_skipped"])
    if total_fused != expected_fused or total_fused - total_published != expected_skipped:
        raise ValueError("mapping.csv fusion/publication counts disagree with summary.yml")

    duration = (end_ns - start_ns) / NS_PER_SECOND
    failure = summary.get("failure", "")
    has_snapshot = summary["has_snapshot"] == "1"
    incomplete_reason = failure
    if not incomplete_reason and input_end_ns < requested_end_ns:
        incomplete_reason = "live input ended before the requested window"
    if not incomplete_reason and not has_snapshot:
        incomplete_reason = "no map snapshot was produced"
    return {
        "status": "incomplete" if incomplete_reason else "complete",
        "requested_seconds": seconds,
        "duration_seconds": duration,
        "mapping_target_hz": float(summary["mapping_target_hz"]),
        "published_updates": published,
        "published_hz": published / duration,
        "fused_updates": fused,
        "fused_hz": fused / duration,
        "published_per_second": per_second,
        "zero_update_seconds": per_second.count(0),
        "has_snapshot": has_snapshot,
        "failure": failure,
        "incomplete_reason": incomplete_reason,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="run output containing mapping.csv and summary.yml")
    parser.add_argument("--seconds", type=int, default=60, help="seconds from the live start")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    try:
        report = measure(args.run, args.seconds)
    except (OSError, KeyError, TypeError, ValueError, csv.Error) as error:
        print(f"Cannot measure mapping rate: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "complete" else 1


if __name__ == "__main__":
    sys.exit(main())
