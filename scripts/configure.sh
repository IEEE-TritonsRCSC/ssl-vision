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
