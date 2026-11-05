"""One module per CLI command family.

This package intentionally holds only the shared error printer. Command-specific helpers live
with the command that owns them, so a new command does not grow a central dispatch file.
"""

from __future__ import annotations

import sys

__all__ = ["print_error"]


def print_error(message: str) -> None:
    """Print a command failure in the CLI's stable ``pysoarm: ...`` format."""
    print(f"pysoarm: {message}", file=sys.stderr)
