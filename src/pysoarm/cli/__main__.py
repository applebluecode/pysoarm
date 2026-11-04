"""Allow ``python -m pysoarm``."""

from __future__ import annotations

import sys

from pysoarm.cli.main import main

if __name__ == "__main__":
    sys.exit(main())
