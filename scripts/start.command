#!/bin/bash
# Double-click in Finder to start AIRFRAME_DESIGNER (PX4 SITL, or the Pixhawk if one is plugged in).
cd "$(dirname "$0")/.." || exit 1
if [ ! -x .venv/bin/python ]; then
  echo "Creating Python environment…"
  python3 -m venv .venv && .venv/bin/pip install -q -e ".[dev]"
fi
exec .venv/bin/python -m airframe_designer ui --mode auto --launch-px4
