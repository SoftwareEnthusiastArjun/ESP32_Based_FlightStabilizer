# build.spec — PyInstaller build configuration for FlightGCS
# Run with:  pyinstaller build.spec

import sys
import os
from PyInstaller.utils.hooks import collect_data_files, collect_submodules

block_cipher = None

# ── Collect all data files ────────────────────────────────────────────────────
datas = [
    # Include the entire gcs/ package
    ("gcs/*.py",        "gcs"),
    # Include the icon
    ("icon.ico",        "."),
]

# ── Hidden imports (modules PyInstaller misses) ───────────────────────────────
hiddenimports = [
    "gcs.theme",
    "gcs.connection",
    "gcs.control_tab",
    "gcs.servo_tab",
    "gcs.visualizer_tab",
    "gcs.pid_tab",
    "gcs.diagnostics_tab",
    "gcs.hud",
    "pygame",
    "pygame.locals",
    "tkinter",
    "tkinter.ttk",
    "tkinter.messagebox",
    "tkinter.filedialog",
    "socket",
    "threading",
    "concurrent.futures",
    "queue",
    "math",
    "datetime",
]

a = Analysis(
    ["main.py"],                      # entry point
    pathex=["."],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        "matplotlib", "numpy", "pandas",
        "scipy", "PIL", "cv2",          # not needed, reduce size
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="FlightGCS",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,                          # compress (install UPX for smaller size)
    console=False,                     # no black console window
    disable_windowed_traceback=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon="icon.ico",                   # taskbar + exe icon
    version="version_info.txt",        # version metadata (created below)
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="FlightGCS",                  # output folder name inside dist/
)
