"""Single source of truth for the runtime version.

``importlib.metadata`` reads it from the installed distribution, so the number in
``pyproject.toml`` is authoritative; a source checkout that was never installed falls back to a
PEP 440 local version instead of failing.
"""

from __future__ import annotations

from importlib import metadata

__all__ = ["__version__"]


def _resolve() -> str:
    try:
        return metadata.version("pysoarm")
    except metadata.PackageNotFoundError:  # running from a source tree
        return "0.0.0+unknown"


__version__ = _resolve()
