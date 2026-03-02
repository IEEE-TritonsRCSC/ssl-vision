# AGENTS.md

This file provides build/lint/test commands and code style guidelines for agentic coding assistants working on this repository.

## Build Commands

### Building the Project
```bash
# Standard build (Release mode)
make
# or
cmake --build build

# Configure with specific camera drivers
cmake -S . -B build -DUSE_SPINNAKER=true
./scripts/configure.sh -DUSE_SPINNAKER=true

# Available camera flags: USE_DC1394, USE_SPINNAKER, USE_mvIMPACT,
# USE_PYLON, USE_FLYCAP, USE_V4L, USE_AVFOUNDATION, USE_SPLITTER
```

### Cleaning
```bash
make clean
make cleanup_cache  # Removes build directory entirely
```

### Running the Applications
```bash
# Main vision application with auto-start capture
make run
LC_NUMERIC=en_US.UTF-8 ./bin/vision -s

# Non-graphical client
make run_client
./bin/client

# Graphical client
make run_graphical_client
./bin/graphicalClient
```

### Test Data
```bash
make install_test_data  # Downloads test images to test-data/
```

### Testing
No automated test framework exists currently. Testing is done manually.

### Linting/Format
```bash
# Format code according to .clang-format rules
clang-format -i <file>
# To format all files:
find src -name "*.cpp" -o -name "*.h" | xargs clang-format -i
```

## Code Style Guidelines

### File Headers
Every source file must include the GPL license header:
```cpp
//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    filename.h
  \brief   Brief description
  \author  Author Name, Year
*/
//========================================================================
```

### Include Guards and Ordering
- Header guards: `__FILENAME_H__` or `FILENAME_HUPPERCASE_H_`
- Include groups (separated by blank lines):
  1. System/C++ standard headers (alphabetical)
  2. Qt headers (alphabetical)
  3. Platform-specific headers (inside `#ifdef`)
  4. Project headers (alphabetical, grouped by directory)
- For .cpp files, include the corresponding header first

Example:
```cpp
#include <string>
#include <vector>
#include <cstdio>

#include <QThread>
#include <QMutex>

#ifdef __linux__
#include <linux/videodev2.h>
#endif

#include "captureinterface.h"
#include "util.h"
```

### Formatting (.clang-format)
- Style: Google-based
- Column limit: 120 characters
- Indentation: 2 spaces (no tabs)
- Braces: Attached (K&R style)
- Pointer alignment: Left (`Type* name`)

### Naming Conventions
- Classes: PascalCase (`CaptureThread`, `VisionStack`)
- Functions: camelCase (`processFrame`, `getSettings`)
- Member variables: camelCase, often with `_` prefix (`_settings`, `capture`)
- Constants: UPPER_CASE or PascalCase depending on context
- Macros: UPPER_CASE with underscores (`V4L_STREAMBUFS`)
- Filenames: lowercase with underscores (`capture_thread.cpp`)

### Type Usage
- Use `std::string`, `std::vector`, etc. from STL
- Use `bool` for boolean values
- Pre-passed by value for small types, by const reference for large objects
- Template parameters: PascalCase (`CLASS_VARVAL_TYPE`)
- Qt types: `QString`, `QList`, etc.

### Qt Integration
- Use `Q_OBJECT` macro for classes that define signals/slots
- Use Qt's parent-child ownership model for memory management
- Signals/slots for async communication: `public slots:`, `signals:`
- Thread-safe operations via `QMutex`

### Preprocessor Conditionals
Group platform or feature-specific code:
```cpp
#ifdef DC1394
  // DC1394-specific code
#endif

#ifdef __linux__
  // Linux-specific code
#endif
```

### Error Handling
- Qt exceptions: Use signals/slots, return values
- System calls: Check return values, use `perror` or fprintf stderr
- Critical errors: `message(FATAL_ERROR "")` in CMake, abort() in code

### Comments and Documentation
- Multi-line `//` style preferred over `/* */` for C++ comments
- Doxygen for public APIs: `/** */` or `/*! */`
- Internal comments: `//` at same indentation level
- No trailing comments on same line as code when possible

### Project Structure
- `src/app/`: Main application code
- `src/shared/`: Shared libraries (capture, util, vartypes, etc.)
- `src/client/`: Non-graphical client
- `src/graphicalClient/`: Graphical client
- `cmake/`: CMake modules
- `scripts/`: Helper scripts

### Build Configuration
- CMake version minimum: 3.16.3
- C++ standard: C++11
- Qt version auto-detect: Qt5 or Qt6
- Platform-specific: macOS uses AVFoundation, Linux uses V4L2
