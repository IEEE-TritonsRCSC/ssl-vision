#!/usr/bin/env bash
set -euo pipefail

if ! command -v brew >/dev/null 2>&1; then
  echo "Homebrew is required but was not found. Install it from https://brew.sh/ and re-run this script." >&2
  exit 1
fi

brew update

BREW_PREFIX="$(brew --prefix)"

BREW_PACKAGES=(
  cmake
  eigen
  protobuf
  qt@6
  opencv@4
  glew
  jpeg-turbo
  libpng
  pkg-config
  libusb
  jsoncpp
  tbb
  boost
  libpcap
  libav
  libdispatch
)

echo "Installing base dependencies..."
for pkg in "${BREW_PACKAGES[@]}"; do
  if brew list --versions "${pkg}" >/dev/null 2>&1; then
    echo "[brew] ${pkg} already installed"
  else
    echo "[brew] Installing ${pkg}..."
    brew install "${pkg}"
  fi
done

# Remove deprecated packages
echo "Checking for deprecated packages..."
DEPRECATED_PACKAGES=(
  freeglut
  libdc1394
  zeroconf
)

for pkg in "${DEPRECATED_PACKAGES[@]}"; do
  if brew list --versions "${pkg}" >/dev/null 2>&1; then
    echo "[brew] Removing deprecated package ${pkg}..."
    brew uninstall --ignore-dependencies "${pkg}" || true
  fi
done

if brew list --versions qt@6 >/dev/null 2>&1; then
  cat <<'EOF'
[info] Qt 6 is installed. CMake will automatically detect and use it if available.
EOF

  # Ensure Qt6 tools are in PATH
  QT6_PATH="$(brew --prefix qt@6)"
  if [ -d "${QT6_PATH}" ]; then
    echo "[info] Qt 6 installed at: ${QT6_PATH}"
    echo "[info] Qt 6 tools: ${QT6_PATH}/bin"
    echo "[info] If CMake fails to find Qt6, configure with:"
    echo "       -DCMAKE_PREFIX_PATH=${QT6_PATH}"
  fi
fi

echo ""
echo "Base dependencies installed successfully."
echo ""
echo "Optional SDKs (Spinnaker, Pylon, mvIMPACT) must be installed manually following vendor instructions."
echo ""
echo "To build ssl-vision on macOS, run:"
echo "  cmake -B build -DUSE_AVFOUNDATION=ON -DCMAKE_PREFIX_PATH=$(brew --prefix qt@6) -DCMAKE_OSX_DEPLOYMENT_TARGET=$(sw_vers -productVersion)"
echo "  cmake --build build"
