# Build Guide (Windows + Linux)

## Important
- A single binary cannot run on both Windows and Linux.
- Build once on each target OS.
- Source code/package is cross-platform.

## 1) Install build dependency
```bash
python -m pip install -r requirements-build.txt
```

## 2) Build binary (same command on Windows/Linux)
```bash
python build_binary.py
```

Output:
- Windows: `dist/dof2-traj-gui/` (contains `dof2-traj-gui.exe`)
- Linux: `dist/dof2-traj-gui/` (contains executable `dof2-traj-gui`)

## Optional: single-file binary
```bash
python build_binary.py --onefile
```

Output:
- Windows: `dist/dof2-traj-gui.exe`
- Linux: `dist/dof2-traj-gui`

## Optional: keep console logs
```bash
python build_binary.py --console
```

## Cross-platform distribution recommendation
- For developers/users with Python: publish package (`pip install .`)
- For end users without Python: publish OS-specific binaries separately

