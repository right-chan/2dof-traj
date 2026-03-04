from __future__ import annotations

import sys
from pathlib import Path

# Allow direct execution from repository root without installation.
ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from dof2_trajectory_gui.main import main


if __name__ == "__main__":
    raise SystemExit(main())
