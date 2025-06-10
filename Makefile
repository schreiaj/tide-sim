.PHONY: install test lint lint-fix format clean

install:
	uv venv
	. .venv/bin/activate && uv pip install -e ".[dev]"

test:
	. .venv/bin/activate && pytest -v --cov

lint:
	. .venv/bin/activate && ruff check .

lint-fix:
	. .venv/bin/activate && ruff check . --fix

format:
	. .venv/bin/activate && black .
	. .venv/bin/activate && isort .

clean:
	rm -rf build/
	rm -rf dist/
	rm -rf *.egg-info
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type d -name .pytest_cache -exec rm -rf {} +
	find . -type d -name .coverage -exec rm -rf {} +
	find . -type d -name htmlcov -exec rm -rf {} + 