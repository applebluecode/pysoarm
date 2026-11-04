# Contributing to pysoarm

Thanks for taking the time to contribute! This document describes the workflow we expect
for issues and pull requests.

## Development setup

```bash
git clone https://github.com/applebluecode/pysoarm && cd pysoarm
make install-dev      # uv sync --locked + pre-commit hooks
```

The dev environment is managed with [uv](https://docs.astral.sh/uv/) and pinned by the
committed `uv.lock`; `pyproject.toml` stays pip-installable for users. Without uv:

```bash
python -m venv .venv && . .venv/bin/activate
pip install --group dev -e .
pre-commit install
```

If you resolve through a local mirror, use `UV_DEFAULT_INDEX=<mirror>/simple uv lock` but
**do not commit that lockfile** — it must keep pointing at `pypi.org` for portability. If
`uv sync --locked` refuses because your machine configures a different default index,
either run it as `UV_DEFAULT_INDEX=https://pypi.org/simple uv sync --locked` or rely on the
project-level index pin in `pyproject.toml` (which takes precedence).

## Before opening a pull request

```bash
make check    # lint + type + test, exactly what CI runs
```

or individually:

```bash
make lint     # ruff check + ruff format --check
make type     # mypy --strict
make test     # pytest (hardware tests are skipped by default)
```

CI runs the same commands on Python 3.10 – 3.13, so a green local run is a good proxy.

## Guidelines

* **Typing is mandatory.** The package ships a `py.typed` marker and is checked with
  `mypy --strict`; public functions need full annotations.
* **Public API changes need a changelog entry** under `## [Unreleased]`.
* **Keep hardware access behind the `bus` Protocol.** Anything that touches `pyserial`
  or `rustypot` belongs in `pysoarm/bus/`, so the rest stays testable without an arm.
* **New behaviour needs tests.** Use the `FakeBus` backend for logic; mark tests that
  require a physical arm with `@pytest.mark.hardware`.
* **Safety-relevant code needs a reviewer's attention.** Changes in `safety.py` or in the
  write path must state which guard they touch and why it cannot cause a runaway joint.
* Write docstrings in Google style; they are rendered by mkdocstrings.
* Prefer small, focused pull requests; one concern per PR.

## Commit messages

[Conventional Commits](https://www.conventionalcommits.org/) are recommended, e.g.
`feat(bus): add SCS sync_write`, `fix(safety): clamp first goal after connect`.

## Reporting issues

Please use the issue templates and include:

* the arm model (SO-100 / SO-101) and servo firmware version if known;
* the serial adapter (`lsusb`, `udevadm info -q property -n /dev/ttyACM0`);
* the output of `pysoarm scan` and your `pysoarm --version`;
* whether another process (LeRobot, `ModemManager`, a suspended `lerobot-*`) holds the port.
