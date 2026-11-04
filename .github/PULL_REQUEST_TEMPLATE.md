## What and why

<!-- One paragraph: what changes, and which problem it solves. Link the issue. -->

Closes #

## Type of change

- [ ] Bug fix (non-breaking)
- [ ] New feature (non-breaking)
- [ ] Breaking change (public API or calibration-format change)
- [ ] Documentation / tooling only

## Checklist

- [ ] `make lint type test` passes locally
- [ ] Public API changes are typed and documented (Google-style docstrings)
- [ ] `CHANGELOG.md` updated under `## [Unreleased]`
- [ ] New behaviour is covered by tests (unit tests use `FakeBus`; hardware tests are marked)
- [ ] No hardware access outside `pysoarm/bus/`

## Safety review (required if this touches `safety.py`, `arm.py` write path, or `stop()`)

- [ ] Which guard does this change, and why can it not command a runaway joint?
- [ ] Is torque released on every error path, including `Exception` and signals?
- [ ] Does `write_goal()` still return the goal actually sent?

## Testing done

<!-- Hardware: arm model, servo firmware, adapter. Or "unit tests only". -->
