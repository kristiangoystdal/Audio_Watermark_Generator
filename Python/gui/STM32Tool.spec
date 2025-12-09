# -*- mode: python ; coding: utf-8 -*-

app_name = "Audio Watermark Flash Tool"

a = Analysis(
    ['gui.py'],
    pathex=[],
    binaries=[
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
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name=app_name,
    debug=False,
    strip=False,
    upx=True,
    console=False,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    name=app_name,
)

app = BUNDLE(
    coll,
    name=f"{app_name}.app",
    icon="icon.icns",
    bundle_identifier="no.ntnu.kristian.stm32tool",
)
