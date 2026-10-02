#!/usr/bin/env python3
"""Sample one Linux process and its descendants; write CSV and a JSON summary."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import time


def process_stats(pid):
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        # comm may contain spaces or parentheses; fields start after its final ')'.
        fields = stat[stat.rfind(")") + 2:].split()
        return {
            "pid": pid, "ppid": int(fields[1]), "start_ticks": int(fields[19]),
            "cpu_ticks": int(fields[11]) + int(fields[12]),
            "rss_bytes": max(0, int(fields[21])) * os.sysconf("SC_PAGE_SIZE"),
        }
    except (OSError, ValueError, IndexError):
        return None


def snapshot(root_pid, descendants):
    root = process_stats(root_pid)
    if not root:
        return {}
    if not descendants:
        return {root_pid: root}
    processes = {}
    for entry in Path("/proc").iterdir():
        if entry.name.isdigit():
            stat = process_stats(int(entry.name))
            if stat:
                processes[stat["pid"]] = stat
    selected = {root_pid}
    while True:
        extra = {pid for pid, stat in processes.items() if stat["ppid"] in selected}
        if extra <= selected:
            break
        selected |= extra
    return {pid: processes[pid] for pid in selected if pid in processes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid", type=int, required=True, help="Launch PID (tree) or node PID")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--output", type=Path, required=True, help="Output CSV; adjacent JSON is also written")
    parser.add_argument("--single-process", action="store_true", help="Exclude descendants")
    args = parser.parse_args()
    if (args.pid <= 0 or args.duration <= 0 or args.interval <= 0 or args.warmup < 0
            or not all(math.isfinite(value) for value in (args.duration, args.interval, args.warmup))):
        parser.error("PID, duration and interval must be positive; warmup must be nonnegative")
    identity = process_stats(args.pid)
    if not identity:
        parser.error("Target process does not exist")
    time.sleep(args.warmup)
    previous = snapshot(args.pid, not args.single_process)
    previous_time = start = time.monotonic()
    clock_ticks = os.sysconf("SC_CLK_TCK")
    rows = []
    per_process = {}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=["elapsed_s", "cpu_percent", "rss_mib", "process_count"])
        writer.writeheader()
        while previous_time - start < args.duration:
            time.sleep(min(args.interval, args.duration - (previous_time - start)))
            now = time.monotonic()
            current = snapshot(args.pid, not args.single_process)
            root = current.get(args.pid)
            if not root or root["start_ticks"] != identity["start_ticks"]:
                break
            delta_seconds = now - previous_time
            ticks = 0
            for pid, stat in current.items():
                before = previous.get(pid)
                if before and before["start_ticks"] == stat["start_ticks"]:
                    delta = max(0, stat["cpu_ticks"] - before["cpu_ticks"])
                elif stat["start_ticks"] / clock_ticks >= float(Path("/proc/uptime").read_text().split()[0]) - delta_seconds:
                    delta = stat["cpu_ticks"]  # Process started within this interval.
                else:
                    delta = 0
                ticks += delta
                key = f"{pid}:{stat['start_ticks']}"
                entry = per_process.setdefault(key, {"pid": pid, "cpu_seconds": 0.0, "peak_rss_mib": 0.0})
                entry["cpu_seconds"] += delta / clock_ticks
                entry["peak_rss_mib"] = max(entry["peak_rss_mib"], stat["rss_bytes"] / 1048576)
                try:
                    entry["command"] = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
                except OSError:
                    pass
            row = {
                "elapsed_s": now - start,
                "cpu_percent": ticks / clock_ticks / delta_seconds * 100,
                "rss_mib": sum(stat["rss_bytes"] for stat in current.values()) / 1048576,
                "process_count": len(current),
            }
            writer.writerow(row)
            output.flush()
            rows.append(row)
            previous, previous_time = current, now
    if not rows:
        raise SystemExit("No samples collected: target exited during warmup or before first sample")
    total_time = rows[-1]["elapsed_s"]
    cpu_seconds = sum(entry["cpu_seconds"] for entry in per_process.values())
    summary = {
        "root_pid": args.pid, "include_descendants": not args.single_process,
        "requested_duration_s": args.duration, "measured_duration_s": total_time,
        "warmup_s": args.warmup, "samples": len(rows),
        "cpu_percent_mean": cpu_seconds / total_time * 100,
        "cpu_percent_peak": max(row["cpu_percent"] for row in rows),
        "rss_mib_mean": statistics.mean(row["rss_mib"] for row in rows),
        "rss_mib_peak": max(row["rss_mib"] for row in rows),
        "cpu_definition": "100% equals one fully occupied CPU core; may exceed 100%",
        "memory_definition": "Sum of RSS; shared pages can be counted more than once",
        "sampling_limit": "Processes that start and exit between samples may be missed",
        "processes": list(per_process.values()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "processes"}, indent=2))


if __name__ == "__main__":
    main()
