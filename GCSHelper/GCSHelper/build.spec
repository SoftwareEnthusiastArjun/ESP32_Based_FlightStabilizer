# build.spec — PyInstaller config for GCSHelper
# Run from GCSHelper/ folder:
#   pyinstaller build.spec --clean

block_cipher = None

a = Analysis(
    ["flight_serial_monitor_gui.py"],
    pathex=["."],
    binaries=[],
    datas=[
        ("flash_tab.py", "."),
        ("icon.ico",     "."),
    ],
    hiddenimports=[
        "serial",
        "serial.tools",
        "serial.tools.list_ports",
        "esptool",
        "tkinter",
        "tkinter.ttk",
        "tkinter.messagebox",
        "tkinter.filedialog",
        "threading",
        "subprocess",
        "queue",
        "importlib",
        "importlib.util",
    ],
    hookspath=[],
    runtime_hooks=[],
    excludes=["matplotlib", "numpy", "pygame"],
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="GCSHelper",
    debug=False,
    strip=False,
    upx=True,
    console=False,
    icon="icon.ico",
    version="version_info.txt",
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    name="GCSHelper",
)
