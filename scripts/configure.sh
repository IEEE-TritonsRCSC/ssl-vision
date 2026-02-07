#!/usr/bin/env bash
set -euo pipefail

# Helper to configure ssl-vision safely across machines.
# It reuses build trees when possible but deletes caches whose
# CMAKE_HOME_DIRECTORY does not match the current checkout.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR%/scripts}"
PROJECT_ROOT="${PROJECT_ROOT:-$PWD}"

BUILD_DIR="build"
SOURCE_DIR="$PROJECT_ROOT"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -B|--build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    -S|--source-dir)
      SOURCE_DIR="$2"
      shift 2
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

has_cmake_flag() {
  local key="$1"
  for arg in "${EXTRA_ARGS[@]}"; do
    if [[ "$arg" == "-D${key}="* ]]; then
      return 0
    fi
  done
  return 1
}

if [[ "$(uname -s)" == "Darwin" ]]; then
  # Avoid stale CMake cache values re-enabling optional camera SDKs on macOS.
  has_cmake_flag "USE_DC1394" || EXTRA_ARGS+=("-DUSE_DC1394=OFF")
  has_cmake_flag "USE_V4L" || EXTRA_ARGS+=("-DUSE_V4L=OFF")
  has_cmake_flag "USE_SPINNAKER" || EXTRA_ARGS+=("-DUSE_SPINNAKER=OFF")
  has_cmake_flag "USE_PYLON" || EXTRA_ARGS+=("-DUSE_PYLON=OFF")
  has_cmake_flag "USE_mvIMPACT" || EXTRA_ARGS+=("-DUSE_mvIMPACT=OFF")
  has_cmake_flag "USE_FLYCAP" || EXTRA_ARGS+=("-DUSE_FLYCAP=OFF")
  has_cmake_flag "USE_AVFOUNDATION" || EXTRA_ARGS+=("-DUSE_AVFOUNDATION=ON")
fi

mkdir -p "$BUILD_DIR"
CACHE_FILE="$BUILD_DIR/CMakeCache.txt"
if [[ -f "$CACHE_FILE" ]]; then
  CACHE_SOURCE="$(grep '^CMAKE_HOME_DIRECTORY:INTERNAL=' "$CACHE_FILE" | head -n1 | cut -d= -f2- || true)"
  if [[ -n "$CACHE_SOURCE" && "$CACHE_SOURCE" != "$SOURCE_DIR" ]]; then
    echo "[configure] Detected stale CMake cache from '$CACHE_SOURCE'. Removing '$BUILD_DIR'."
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
  fi
fi

cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" "${EXTRA_ARGS[@]}"
