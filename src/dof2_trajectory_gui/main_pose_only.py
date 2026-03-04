from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from .gui_pose_only import PoseOnlyWindow


def main() -> int:
    app = QApplication(sys.argv)
    window = PoseOnlyWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

