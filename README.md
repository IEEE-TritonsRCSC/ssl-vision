# RoboCup Small Size League Shared Vision System (macOS)

This repository contains `ssl-vision` configured for **macOS-only** development and usage.

## Supported Environment

- macOS 13+ (Apple Silicon or Intel)
- Xcode Command Line Tools
- Homebrew
- CMake 3.16+
- Qt 6 (via Homebrew)

## 1) Install Dependencies (macOS)

Run the provided installer:

```bash
./InstallPackagesMac.sh
```

It installs and/or verifies the Homebrew dependencies required for this repo.

## 2) Configure a Clean Build

Use the configure helper script so stale CMake cache entries are handled automatically.

```bash
./scripts/configure.sh \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_DC1394=OFF \
  -DUSE_V4L=OFF \
  -DUSE_AVFOUNDATION=ON \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)" \
  -DCMAKE_OSX_DEPLOYMENT_TARGET="$(sw_vers -productVersion)"
```

Notes:

- `USE_AVFOUNDATION=ON` is the macOS camera backend.
- `USE_DC1394=OFF` and `USE_V4L=OFF` are required for normal macOS builds.

## 3) Build

```bash
cmake --build build -j
```

Binaries are written to:

- `./bin/vision`
- `./bin/client`
- `./bin/graphicalClient`

## 4) Run

Main app:

```bash
LC_NUMERIC=en_US.UTF-8 ./bin/vision -s
```

Non-graphical client:

```bash
./bin/client
```

Graphical client:

```bash
./bin/graphicalClient
```

## 5) Optional: Test Data

```bash
make install_test_data
```

This downloads sample frames into `./test-data`.

## Clean Rebuild

If you want a full clean rebuild:

```bash
rm -rf build bin
./scripts/configure.sh \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_DC1394=OFF \
  -DUSE_V4L=OFF \
  -DUSE_AVFOUNDATION=ON \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)" \
  -DCMAKE_OSX_DEPLOYMENT_TARGET="$(sw_vers -productVersion)"
cmake --build build -j
```

## Troubleshooting (macOS)

- Camera access denied:
  - Enable camera permission for the terminal/app running `ssl-vision` in System Settings.
- CMake cache mismatch across checkouts:
  - Always use `./scripts/configure.sh` rather than direct `cmake -B build`.
- Locale decimal separator issues in XML:
  - Run `LC_NUMERIC=en_US.UTF-8` when starting `vision`.

## Optional Camera SDKs

Spinnaker, Pylon, and mvIMPACT are optional and must be installed from vendor packages first. After installation, enable with CMake flags such as:

- `-DUSE_SPINNAKER=ON`
- `-DUSE_PYLON=ON`
- `-DUSE_mvIMPACT=ON`
