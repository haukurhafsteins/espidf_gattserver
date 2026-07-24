#!/usr/bin/env python3

from pathlib import Path
import sys


SOURCE = Path(__file__).resolve().parents[1] / "gatt.cpp"


def main() -> int:
    source = SOURCE.read_text(encoding="utf-8")
    start = source.find("esp_err_t gatt_notify_custom(")
    end = source.find("\nesp_err_t gatt_set_value(", start)
    if start < 0 or end < 0:
        print("custom notify implementation not found", file=sys.stderr)
        return 1

    implementation = source[start:end]
    requirements = ("ble_hs_mbuf_from_flat", "ble_gatts_notify_custom")
    for marker in requirements:
        if marker not in implementation:
            print(f"custom notify missing {marker}", file=sys.stderr)
            return 1
    if "gatt_set_value(" in implementation:
        print("custom notify must not change the readable value", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
