#!/usr/bin/env python3
"""Fail if .text plus .data of the portable firmware exceeds the committed budget.

Host numbers are a regression proxy. The real Cortex-M4 flash figure comes from
the optional arm-none-eabi-gcc job in CI and is reported, not enforced here.
"""

import pathlib
import subprocess
import sys


def budget_bytes(baseline_path: str) -> int:
    for line in pathlib.Path(baseline_path).read_text().splitlines():
        if line.startswith("text_plus_data_max_bytes="):
            return int(line.split("=", 1)[1])
    raise SystemExit(f"no budget line found in {baseline_path}")


def text_plus_data_bytes(binary_path: str) -> int:
    result = subprocess.run(
        ["size", "-A", binary_path], check=True, capture_output=True, text=True
    )
    total = 0
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] in (".text", ".data"):
            total += int(parts[1])
    return total


def main() -> int:
    if len(sys.argv) != 3:
        raise SystemExit("usage: check_size.py <binary> <baseline>")
    used = text_plus_data_bytes(sys.argv[1])
    budget = budget_bytes(sys.argv[2])
    print(f"firmware_portable .text + .data = {used} bytes, budget {budget} bytes")
    return 0 if used <= budget else 1


if __name__ == "__main__":
    sys.exit(main())
