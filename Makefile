PY := .venv/bin/python

.PHONY: setup ui test test-fast run-hover study clean

setup:
	python3 -m venv .venv && .venv/bin/pip install -q -e ".[dev]"

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
