#!/bin/sh
# Fails if firmware source includes anything except hal.h and the C standard
# library, or if the compiled objects reference anything except hal_ symbols
# and compiler runtime symbols. This is the portability gate, not a promise.
set -eu

if [ "$#" -lt 2 ]; then
  echo "usage: check_firmware_boundary.sh <firmware_dir> <object> [object...]" >&2
  exit 2
fi

firmware_dir="$1"
shift

bad_includes=$(
  grep -RhE '^[[:space:]]*#[[:space:]]*include' "$firmware_dir/src" "$firmware_dir/include" |
    grep -vE '"(hal|firmware)\.h"|<(stdbool|stddef|stdint)\.h>' || true
)
if [ -n "$bad_includes" ]; then
  echo "firmware includes a header it is not allowed to include:" >&2
  echo "$bad_includes" >&2
  exit 1
fi

for object in "$@"; do
  undefined=$(nm -u "$object" | awk '{print $NF}')
  for symbol in $undefined; do
    case "$symbol" in
      hal_*) ;;
      __*) ;;
      memcpy | memset | memmove) ;;
      *)
        echo "firmware references a non HAL symbol: $symbol" >&2
        exit 1
        ;;
    esac
  done
done

echo "firmware boundary: ok"
