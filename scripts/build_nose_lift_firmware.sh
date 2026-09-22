#!/bin/bash
# Build PX4 v1.17.0 with the nose-lift module (firmware/px4_ext) and the control-allocator hook (firmware/patches),
# for SITL or for the flight controller (with the HIL output driver, so the same image flies HITL and the aircraft).
#
#   scripts/build_nose_lift_firmware.sh sitl                       SITL binary for the app: PX4_DIR=~/PX4-nl
#   scripts/build_nose_lift_firmware.sh board [build|upload]       px4_fmu-v6x_multicopter image (flash with QGC)
#
# The tree is a dedicated worktree (default ~/PX4-nl) of ~/PX4-Autopilot at v1.17.0, created on first use; the
# patches stay applied there. Never build this in ~/PX4-Autopilot (shared, patched by other projects) or in
# ~/PX4-hitl (the stock HITL image).
set -euo pipefail
TARGET_KIND="${1:-sitl}"
ACTION="${2:-build}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-nl}"
PX4_REF="${PX4_REF:-v1.17.0}"
BOARD="${BOARD:-px4_fmu-v6x}"
VARIANT="${VARIANT:-multicopter}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
export PATH="${VENV_BIN:-$HOME/.venvs/airframe/bin}:$PATH"
for d in "$HOME"/toolchains/*/bin; do [ -d "$d" ] && export PATH="$d:$PATH"; done

if [ ! -d "$PX4_DIR" ]; then
  echo "creating worktree $PX4_DIR at $PX4_REF"
  git -C "$HOME/PX4-Autopilot" worktree add --detach "$PX4_DIR" "$PX4_REF"
  git -C "$PX4_DIR" submodule update --init --recursive --jobs 8
fi
cd "$PX4_DIR"

for p in "$HERE"/firmware/patches/*.patch; do
  if git apply --check -R "$p" 2>/dev/null; then
    echo "already applied: $(basename "$p")"
  else
    echo "applying $(basename "$p")"
    git apply "$p"
  fi
done

# the external module tree is copied next to the sources: builds from the Windows filesystem are slow and its
# timestamps confuse make
EXT="$PX4_DIR/airframe_designer_ext"
rsync -a --delete "$HERE/firmware/px4_ext/" "$EXT/"

case "$TARGET_KIND" in
  sitl)
    make px4_sitl_default EXTERNAL_MODULES_LOCATION="$EXT"
    echo
    echo "SITL: $PX4_DIR/build/px4_sitl_default/bin/px4   (run the app or 'run' with PX4_DIR=$PX4_DIR or --px4-dir)"
    ;;
  board)
    CFG="boards/${BOARD/_//}/$VARIANT.px4board"
    grep -q "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" "$CFG" || echo "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" >> "$CFG"
    TARGET="${BOARD}_${VARIANT}"
    make "$TARGET" EXTERNAL_MODULES_LOCATION="$EXT"
    OUT="build/$TARGET/$TARGET.px4"
    echo
    echo "Firmware: $PX4_DIR/$OUT"
    if [ "$ACTION" = "upload" ]; then
      # keep `usbipd attach --wsl --busid <id> --auto-attach` running: the board re-enumerates into the bootloader
      echo "Uploading $OUT over USB; if it waits, unplug and replug the board."
      python3 Tools/px_uploader.py --port "/dev/tty.usbmodemPX*,/dev/tty.usbmodem*,/dev/ttyACM*" "$OUT"
    fi
    ;;
  *)
    echo "usage: $0 sitl | board [build|upload]"; exit 1 ;;
esac
