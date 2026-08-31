#!/usr/bin/env python3

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    implementation = (ROOT / "gatt.cpp").read_text(encoding="utf-8")
    summary_start = implementation.find(
        "case GattNotifyLogLimiter::Decision::suppressionSummary:"
    )
    summary_end = implementation.find(
        "case GattNotifyLogLimiter::Decision::suppressed:", summary_start
    )
    if summary_start < 0 or summary_end < 0:
        print("notify suppression summary case not found", file=sys.stderr)
        return 1
    if '"Notify rc' not in implementation[summary_start:summary_end]:
        print("notify suppression summary must retain the Notify rc marker", file=sys.stderr)
        return 1

    limiter = (ROOT / "gatt_notify_log_limiter.hpp").read_text(encoding="utf-8")
    required_lock_markers = (
        "portMUX_TYPE",
        "portENTER_CRITICAL",
        "portEXIT_CRITICAL",
        "std::lock_guard",
    )
    for marker in required_lock_markers:
        if marker not in limiter:
            print(
                f"notify limiter missing serialization contract marker: {marker}",
                file=sys.stderr,
            )
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
