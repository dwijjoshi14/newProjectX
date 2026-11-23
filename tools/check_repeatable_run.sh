#!/bin/sh
# Runs the deterministic harness twice and fails if the two runs differ by one
# byte. This is the determinism claim in docs/spec.md, made checkable: same
# compiler, same flags, same architecture. It does not claim cross compiler bit
# identity, which is a different and much stronger statement.
set -eu

if [ "$#" -lt 1 ]; then
  echo "usage: check_repeatable_run.sh <harness> [scenario]" >&2
  exit 2
fi

harness="$1"
scenario="${2:-speed_step}"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

"$harness" --scenario "$scenario" --out "$work/first.csv" > "$work/first.log"
"$harness" --scenario "$scenario" --out "$work/second.csv" > "$work/second.log"

if ! cmp -s "$work/first.csv" "$work/second.csv"; then
  echo "repeated run of $scenario produced different CSV output" >&2
  exit 1
fi

# The console summary is compared too, with the one line that names the output
# file removed, because that line is the only thing a repeat run is expected to
# change.
grep -v '  csv' "$work/first.log" > "$work/first.norm"
grep -v '  csv' "$work/second.log" > "$work/second.norm"
if ! cmp -s "$work/first.norm" "$work/second.norm"; then
  echo "repeated run of $scenario produced different console output" >&2
  exit 1
fi

echo "deterministic repeat: $scenario identical over two runs"
