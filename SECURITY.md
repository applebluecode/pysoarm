# Security policy

## Supported versions

`pysoarm` is pre-1.0; only the latest released version receives security fixes.

## Reporting a vulnerability

Please **do not** open a public issue for security-relevant reports. Use GitHub's private
vulnerability reporting instead:

<https://github.com/applebluecode/pysoarm/security/advisories/new>

Alternatively email `singlecycle27@163.com`. Include a description, a reproduction if possible, and the
version affected. You can expect
an acknowledgement within 72 hours.

## Scope: physical safety is part of security here

This library drives a physical manipulator. Reports about the following are treated with the
same priority as memory-safety issues:

* a write path that can command a goal position outside the calibrated range;
* a missing or ineffective watchdog after bus communication loss;
* `stop()` / `__exit__` paths that leave torque enabled;
* a failure mode where a raised exception leaves the arm driving toward a stale goal.
