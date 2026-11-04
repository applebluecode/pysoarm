.DEFAULT_GOAL := help

# Dependency management: uv (https://docs.astral.sh/uv/). uv.lock is committed, so the dev
# environment is reproducible; the package itself stays pip-installable from pyproject.toml.
UV ?= uv
SRC := src/pysoarm
RUFF_TARGETS := $(SRC) tests examples ros2

.PHONY: help
help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2}'

.PHONY: sync
sync: ## Create/refresh .venv from uv.lock (fails if the lock is stale)
	$(UV) sync --locked

.PHONY: lock
lock: ## Re-resolve dependencies and rewrite uv.lock
	$(UV) lock

.PHONY: upgrade
upgrade: ## Bump locked dependencies to their newest allowed versions
	$(UV) lock --upgrade

.PHONY: install-dev
install-dev: sync ## Dev environment + git hooks
	$(UV) run pre-commit install

.PHONY: install
install: ## Install the package for use (pip path, no dev tooling)
	python -m pip install .

.PHONY: lint
lint: ## Check formatting and lint rules
	$(UV) run ruff check $(RUFF_TARGETS)
	$(UV) run ruff format --check $(RUFF_TARGETS)

.PHONY: format
format: ## Apply formatting and autofixes
	$(UV) run ruff check --fix $(RUFF_TARGETS)
	$(UV) run ruff format $(RUFF_TARGETS)

.PHONY: type
type: ## Static type check (strict)
	$(UV) run mypy

.PHONY: test
test: ## Run the test suite (no hardware)
	$(UV) run pytest

.PHONY: cov
cov: ## Run tests with a coverage report
	$(UV) run pytest --cov=pysoarm --cov-report=term-missing --cov-report=xml

.PHONY: test-hardware
test-hardware: ## Run hardware tests against PYSOARM_HARDWARE_PORT
	PYSOARM_HARDWARE_PORT=$${PYSOARM_HARDWARE_PORT:-/dev/ttyACM0} $(UV) run pytest -m hardware

.PHONY: check
check: lint type test ## Everything CI runs, locally

.PHONY: build
build: clean ## Build sdist + wheel and validate metadata
	$(UV) run python -m build
	$(UV) run twine check --strict dist/*

.PHONY: docs
docs: ## Serve the documentation locally
	$(UV) run --group docs mkdocs serve

.PHONY: docs-build
docs-build: ## Build the docs exactly as CI does (strict)
	$(UV) run --group docs mkdocs build --strict

.PHONY: clean
clean: ## Remove build artefacts and caches
	rm -rf build dist .pytest_cache .mypy_cache .ruff_cache htmlcov .coverage coverage.xml site
	find . -name '__pycache__' -type d -prune -exec rm -rf {} +
