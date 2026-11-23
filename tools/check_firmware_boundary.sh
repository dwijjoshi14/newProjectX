#!/bin/sh
# Fails if firmware source includes anything except its own headers, hal.h and
# three standard headers, or if the compiled objects reference anything except
# hal_ symbols and compiler runtime symbols. This is the portability gate, not
# a promise.
set -eu

if [ "$#" -lt 2 ]; then
  echo "usage: check_firmware_boundary.sh <firmware_dir> <object> [object...]" >&2
  exit 2
fi

firmware_dir="$1"
shift

# The firmware may include its own headers, hal.h and the three standard
# headers it uses. A literal list of header names was a standing invitation to
# add one more name whenever a new internal header appeared, which would have
# made the gate a record of what the code already did. The rule is the
# directory now, and the directory is what gets checked.
bad_includes=$(
  grep -RhE '^[[:space:]]*#[[:space:]]*include' "$firmware_dir/src" "$firmware_dir/include" |
    while IFS= read -r line; do
      case "$line" in
        *'<stdbool.h>'* | *'<stddef.h>'* | *'<stdint.h>'*)
          continue
          ;;
        *'"'*)
          header=${line#*\"}
          header=${header%%\"*}
          if [ -f "$firmware_dir/include/$header" ]; then
            continue
          fi
          printf '%s\n' "$line"
          ;;
        *)
          printf '%s\n' "$line"
          ;;
      esac
    done
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
