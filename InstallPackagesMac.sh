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
  opencv
  glew
  freeglut
  jpeg-turbo
  libpng
  pkg-config
  libusb
  libdc1394
  jsoncpp
  tbb
  boost
  libpcap
  zeroconf
)

for pkg in "${BREW_PACKAGES[@]}"; do
  if brew list --versions "${pkg}" >/dev/null 2>&1; then
    echo "[brew] ${pkg} already installed"
  else
    brew install "${pkg}"
  fi
done

if brew list --versions qt@6 >/dev/null 2>&1; then
  cat <<'EOF'
[info] Qt 6 is installed. CMake will automatically detect and use it if available.
EOF
fi

echo "\nBase dependencies installed. Optional SDKs (Spinnaker, Pylon, mvIMPACT) must be installed manually following vendor instructions."
