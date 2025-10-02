# -*- mode: python ; coding: utf-8 -*-

a = Analysis(
    ['gui.py'],
    pathex=[],
    binaries=[
        # Explicit binaries (executables) so PyInstaller preserves permissions
        ('tools/openocd/bin/openocd', 'tools/openocd/bin'),
        ('tools/ninja/ninja', 'tools/ninja'),
        ('tools/cmake/4.1.2/bin/cmake', 'tools/cmake/4.1.2/bin'),
        ('tools/arm-none-eabi-gcc', 'tools'),
    ],
    datas=[
        ('../../STM32', 'STM32'),
        ('tools', 'tools'),
        ('tools/openocd/share/openocd/scripts', 'tools/openocd/share/openocd/scripts'),
    ],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='STM32Tool',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='STM32Tool',
)

app = BUNDLE(
    coll,
    name='STM32Tool.app',
    icon=None,
    bundle_identifier=None,
)
