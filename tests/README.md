# Tests

```bash
uv run pytest                        # unit tests only (fast, no hardware)
uv run pytest -m hardware             # opt-in smoke tests against a real arm
PYSOARM_HARDWARE_PORT=/dev/ttyACM0 uv run pytest -m hardware
uv run pytest --cov=pysoarm           # with coverage
```

or through the Makefile: `make test`, `make cov`, `make test-hardware`.

Conventions:

* unit tests never open a serial port — use `FakeBus`;
* anything touching real hardware is marked `@pytest.mark.hardware` and skipped by default;
* `filterwarnings = ["error"]` is on: warnings are bugs here.
