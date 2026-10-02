PY := .venv/bin/python

.PHONY: setup ui test test-fast run-hover study clean

setup:
	# uv brings Python 3.12 where the system python3 is older (macOS ships 3.9)
	if command -v uv >/dev/null; then uv venv --python 3.12 .venv && uv pip install -q --python .venv/bin/python -e ".[dev]"; \
	else python3 -m venv .venv && .venv/bin/pip install -q -e ".[dev]"; fi

ui:
	$(PY) -m airframe_designer ui

test-fast:
	$(PY) -m pytest -q -m "not px4"

test:
	$(PY) -m pytest -q

run-hover:
	$(PY) -m airframe_designer run --airframe airframes/atlas_08.json --scenario hover --out results/hover.json

study:
	$(PY) -m airframe_designer study --spec studies/atlas08_hover_tilt.json --workers 6

clean:
	rm -rf results/*/ .pytest_cache; find . -name __pycache__ -type d -exec rm -rf {} +
