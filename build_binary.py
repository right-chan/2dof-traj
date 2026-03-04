from __future__ import annotations

import argparse
import platform
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build a platform-specific binary for dof2-trajectory-planner."
    )
    parser.add_argument(
        "--onefile",
        action="store_true",
        help="Build a single-file binary (slower startup, simpler distribution).",
    )
    parser.add_argument(
        "--console",
        action="store_true",
        help="Show console window (default: windowed/no console).",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    os_name = platform.system().lower()
    app_name = "dof2-traj-gui"

    cmd: list[str] = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--clean",
        "--name",
        app_name,
        "--paths",
        "src",
        "run_gui.py",
    ]

    if args.onefile:
        cmd.append("--onefile")
    if args.console:
        cmd.append("--console")
    else:
        cmd.append("--windowed")

    print(f"[build] platform={os_name}")
    print("[build] command:", " ".join(cmd))
    subprocess.run(cmd, cwd=root, check=True)

    if args.onefile:
        output = root / "dist" / (app_name + (".exe" if os_name == "windows" else ""))
    else:
        output = root / "dist" / app_name
    print(f"[build] done: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

