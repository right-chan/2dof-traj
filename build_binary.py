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
    parser.add_argument(
        "--app",
        choices=["traj", "pose", "all"],
        default="traj",
        help="Select app target: traj(default), pose, or all.",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    os_name = platform.system().lower()

    targets = {
        "traj": ("dof2-traj-gui", "run_gui.py"),
        "pose": ("dof2-pose-gui", "run_pose_only.py"),
    }
    selected = list(targets.keys()) if args.app == "all" else [args.app]

    print(f"[build] platform={os_name}")
    for key in selected:
        app_name, entry_script = targets[key]
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
            entry_script,
        ]

        if args.onefile:
            cmd.append("--onefile")
        if args.console:
            cmd.append("--console")
        else:
            cmd.append("--windowed")

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
