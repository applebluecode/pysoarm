# Testing

```bash
make check          # lint + type + test (what CI runs)
make lint           # ruff check + ruff format --check
make type           # mypy --strict
make test           # unit tests, no hardware
make cov            # + coverage report
make test-hardware  # PYSOARM_HARDWARE_PORT=/dev/ttyACM0
```

All targets go through `uv run`, using the environment pinned in `uv.lock`
(`make sync` refreshes it; `uv sync --locked` fails if the lock is stale).

## Test strategy

* **Transport** (`bus/scs.py`, `bus/rustypot.py`) — covered by protocol-level tests with a
  fake serial peer, plus opt-in hardware smoke tests.
* **Everything above the transport** — runs against `FakeBus`, including fault injection
  (`fail_next_ops`, `latency_s`, wrong checksum, missing reply). No hardware, no sleeps.
* **Safety guards** — pure functions, table-driven tests (clip, clamp, thermal, watchdog).
* **Timing** — one hardware test asserts the read+write step stays under 3 ms, so a
  regression in transaction handling fails CI instead of showing up as jitter in the lab.

`filterwarnings = ["error"]` is enabled project-wide: a `DeprecationWarning` from `pyserial`
or a numpy warning is a test failure.
