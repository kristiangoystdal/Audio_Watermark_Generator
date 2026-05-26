#!/bin/bash
set -e

# Where this script lives (Python/gui)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLS_DIR="${SCRIPT_DIR}/tools"

# Toolchain version + download info
TOOLCHAIN_VERSION=13.2.rel1
TOOLCHAIN_NAME=arm-gnu-toolchain-${TOOLCHAIN_VERSION}-darwin-arm64-arm-none-eabi
TOOLCHAIN_TAR=${TOOLCHAIN_NAME}.tar.xz
TOOLCHAIN_URL=https://developer.arm.com/-/media/Files/downloads/gnu/${TOOLCHAIN_VERSION}/binrel/${TOOLCHAIN_TAR}

# Destination
DEST_DIR="${TOOLS_DIR}/arm-none-eabi-gcc"

echo "👉 Setting up ARM GNU Toolchain in ${DEST_DIR}"

# Make sure tools folder exists
mkdir -p "${TOOLS_DIR}"

# Remove old version if exists
rm -rf "${DEST_DIR}"

# Download tarball if missing
if [ ! -f "${TOOLS_DIR}/${TOOLCHAIN_TAR}" ]; then
    echo "⬇️  Downloading ${TOOLCHAIN_TAR} ..."
    curl -L -o "${TOOLS_DIR}/${TOOLCHAIN_TAR}" "${TOOLCHAIN_URL}"
else
    echo "✅ Already downloaded: ${TOOLCHAIN_TAR}"
fi

# Extract inside tools/
echo "📦 Extracting..."
tar -xf "${TOOLS_DIR}/${TOOLCHAIN_TAR}" -C "${TOOLS_DIR}"

# Rename to standard folder name
mv "${TOOLS_DIR}/${TOOLCHAIN_NAME}" "${DEST_DIR}"

echo "✅ Toolchain ready at: ${DEST_DIR}/bin/arm-none-eabi-gcc"
