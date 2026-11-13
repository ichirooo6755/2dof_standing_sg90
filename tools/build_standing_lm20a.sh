#!/bin/sh
# Build the LM20A recovery firmware at 128 MHz. Upload only when requested.
set -eu

case "${1:-}" in
  "") [ "$#" -eq 0 ] || exit 2 ;;
  --upload) [ "$#" -eq 2 ] || { echo "Usage: $0 [--upload SERIAL_PORT]" >&2; exit 2; } ;;
  *) echo "Usage: $0 [--upload SERIAL_PORT]" >&2; exit 2 ;;
esac

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_root=${SG90_LM20A_BUILD_DIR:-"${TMPDIR:-/tmp}/sg90-lm20a-standing"}
staged_sketch="$build_root/src/standing_xiao"
build_path="$build_root/build"
fqbn='nrf54l15clean:nrf54l15clean:xiao_nrf54lm20b:clean_ble=off,cpu_freq=128m'

# Stage just the sketch and its headers: archived deployments are not inputs.
mkdir -p "$staged_sketch" "$build_path"
cp "$project_root/firmware/standing_xiao/standing_xiao.ino" "$staged_sketch/"
cp "$project_root"/firmware/standing_xiao/*.h "$staged_sketch/"
arduino-cli compile --fqbn "$fqbn" --build-property build.power_flags= \
  --build-path "$build_path" "$staged_sketch"

if [ "${1:-}" = "--upload" ]; then
  arduino-cli upload --fqbn "$fqbn" --port "$2" \
    --input-dir "$build_path" "$staged_sketch"
fi
