#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLS_DIR="${SCRIPT_DIR}/tools"
mkdir -p "${TOOLS_DIR}"

# ─────────────────────────────────────────────
# ARM GNU Toolchain
# ─────────────────────────────────────────────
TOOLCHAIN_VERSION=13.2.rel1
TOOLCHAIN_NAME=arm-gnu-toolchain-${TOOLCHAIN_VERSION}-darwin-arm64-arm-none-eabi
TOOLCHAIN_TAR=${TOOLCHAIN_NAME}.tar.xz
TOOLCHAIN_URL=https://developer.arm.com/-/media/Files/downloads/gnu/${TOOLCHAIN_VERSION}/binrel/${TOOLCHAIN_TAR}
TOOLCHAIN_DEST="${TOOLS_DIR}/arm-none-eabi-gcc"

if [ ! -f "${TOOLCHAIN_DEST}/bin/arm-none-eabi-gcc" ]; then
    echo ">>> ARM GNU Toolchain"
    rm -rf "${TOOLCHAIN_DEST}"
    if [ ! -f "${TOOLS_DIR}/${TOOLCHAIN_TAR}" ]; then
        echo "  Downloading ${TOOLCHAIN_TAR}..."
        curl -L -o "${TOOLS_DIR}/${TOOLCHAIN_TAR}" "${TOOLCHAIN_URL}"
    fi
    echo "  Extracting..."
    tar -xf "${TOOLS_DIR}/${TOOLCHAIN_TAR}" -C "${TOOLS_DIR}"
    mv "${TOOLS_DIR}/${TOOLCHAIN_NAME}" "${TOOLCHAIN_DEST}"
    echo "  Done: ${TOOLCHAIN_DEST}/bin/arm-none-eabi-gcc"
else
    echo "OK  ARM GNU Toolchain"
fi

# ─────────────────────────────────────────────
# CMake 4.1.2
# ─────────────────────────────────────────────
CMAKE_VERSION=4.1.2
CMAKE_NAME=cmake-${CMAKE_VERSION}-macos-universal
CMAKE_TAR=${CMAKE_NAME}.tar.gz
CMAKE_URL=https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_TAR}
CMAKE_DEST="${TOOLS_DIR}/cmake/${CMAKE_VERSION}"

if [ ! -f "${CMAKE_DEST}/bin/cmake" ]; then
    echo ">>> CMake ${CMAKE_VERSION}"
    rm -rf "${CMAKE_DEST}"
    mkdir -p "${TOOLS_DIR}/cmake"
    if [ ! -f "${TOOLS_DIR}/${CMAKE_TAR}" ]; then
        echo "  Downloading ${CMAKE_TAR}..."
        curl -L -o "${TOOLS_DIR}/${CMAKE_TAR}" "${CMAKE_URL}"
    fi
    echo "  Extracting..."
    tar -xf "${TOOLS_DIR}/${CMAKE_TAR}" -C "${TOOLS_DIR}"
    # The macOS tarball nests binaries inside CMake.app; normalise to a flat unix layout
    EXTRACTED="${TOOLS_DIR}/${CMAKE_NAME}"
    if [ -d "${EXTRACTED}/CMake.app/Contents" ]; then
        mv "${EXTRACTED}/CMake.app/Contents" "${CMAKE_DEST}"
        rm -rf "${EXTRACTED}"
    else
        mv "${EXTRACTED}" "${CMAKE_DEST}"
    fi
    echo "  Done: ${CMAKE_DEST}/bin/cmake"
else
    echo "OK  CMake ${CMAKE_VERSION}"
fi

# ─────────────────────────────────────────────
# Ninja 1.13.1
# ─────────────────────────────────────────────
NINJA_VERSION=1.13.1
NINJA_ZIP=ninja-mac.zip
NINJA_URL=https://github.com/ninja-build/ninja/releases/download/v${NINJA_VERSION}/${NINJA_ZIP}
NINJA_DEST="${TOOLS_DIR}/ninja/ninja"

if [ ! -f "${NINJA_DEST}" ]; then
    echo ">>> Ninja ${NINJA_VERSION}"
    mkdir -p "${TOOLS_DIR}/ninja"
    if [ ! -f "${TOOLS_DIR}/${NINJA_ZIP}" ]; then
        echo "  Downloading ${NINJA_ZIP}..."
        curl -L -o "${TOOLS_DIR}/${NINJA_ZIP}" "${NINJA_URL}"
    fi
    echo "  Extracting..."
    unzip -o "${TOOLS_DIR}/${NINJA_ZIP}" -d "${TOOLS_DIR}/ninja"
    chmod +x "${NINJA_DEST}"
    echo "  Done: ${NINJA_DEST}"
else
    echo "OK  Ninja ${NINJA_VERSION}"
fi

# ─────────────────────────────────────────────
# dfu-util 0.11  (requires libusb — installed via Homebrew)
# ─────────────────────────────────────────────
DFU_DEST="${TOOLS_DIR}/dfu-util/dfu-util"

if [ ! -f "${DFU_DEST}" ]; then
    echo ">>> dfu-util"
    mkdir -p "${TOOLS_DIR}/dfu-util"
    if ! command -v brew &>/dev/null; then
        echo "  ERROR: Homebrew not found. Install it from https://brew.sh then re-run."
        exit 1
    fi
    brew install dfu-util
    BREW_DFU="$(brew --prefix dfu-util)/bin/dfu-util"
    cp "${BREW_DFU}" "${DFU_DEST}"
    echo "  Done: ${DFU_DEST}"
else
    echo "OK  dfu-util"
fi

echo ""
echo "All tools ready in ${TOOLS_DIR}"
