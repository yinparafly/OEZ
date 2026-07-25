# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller: ABI 监控 + 曲线工作室 → 单文件 exe。"""

block_cipher = None

a = Analysis(
    ['abi_monitor.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[
        'ble_link',
        'curve_studio',
        'rpm_chart',
        'snap_edit',
        'voice_notify',
        'abi_sim',
        'serial',
        'serial.tools',
        'serial.tools.list_ports',
        'bleak',
        'bleak.backends',
        'bleak.backends.winrt',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['matplotlib', 'numpy', 'PIL', 'pytest'],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='AbiRpmMonitor',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
