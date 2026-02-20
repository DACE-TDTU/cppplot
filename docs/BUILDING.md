# How to Build CppPlot

## Prerequisites

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.14 or higher (optional, for building examples/tests)

## Method 1: Header-Only (Simplest)

CppPlot is a header-only library. Just copy the `include/cppplot` folder to your project and include it:

```cpp
#include "cppplot/cppplot.hpp"
```

Compile with C++17:

```bash
# Windows (MinGW)
g++ -std=c++17 -I/path/to/cppplot/include your_file.cpp -o your_program

# Linux/Mac
g++ -std=c++17 -I/path/to/cppplot/include your_file.cpp -o your_program -lm
```

## Method 2: CMake Integration

### As Subdirectory

Add CppPlot to your project:

```cmake
add_subdirectory(cppplot)
target_link_libraries(your_target PRIVATE cppplot)
```

### Using FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(
    cppplot
    GIT_REPOSITORY https://github.com/yourusername/cppplot.git
    GIT_TAG main
)
FetchContent_MakeAvailable(cppplot)
target_link_libraries(your_target PRIVATE cppplot)
```

## Building Examples and Tests

```bash
# Create build directory
mkdir build
cd build

# Configure
cmake ..

# Build
cmake --build .

# Run tests
ctest

# Run examples
./examples/basic_plot
./examples/subplots
./examples/scientific
```

## Windows Specific

### With MinGW

```bash
g++ -std=c++17 -D_USE_MATH_DEFINES -Iinclude examples/quick_start.cpp -o quick_start.exe
```

### With MSVC

```bash
cl /std:c++17 /EHsc /Iinclude examples/quick_start.cpp
```

### With Visual Studio

1. Create new C++ project
2. Add `include` folder to Include Directories
3. Set C++ Language Standard to C++17
4. Add source files and build

## Troubleshooting

### M_PI not defined

Add before including cppplot:

```cpp
#define _USE_MATH_DEFINES
#include <cmath>
```

Or use CMake definition:

```cmake
target_compile_definitions(your_target PRIVATE _USE_MATH_DEFINES)
```

### Linker errors on Linux

Make sure to link math library:

```bash
g++ -std=c++17 -Iinclude your_file.cpp -o your_program -lm
```

### Unicode issues on Windows

Ensure your terminal supports UTF-8 or use ASCII-only labels.
