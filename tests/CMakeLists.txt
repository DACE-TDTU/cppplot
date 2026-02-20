# Tests CMakeLists.txt

# Test core functionality
add_executable(test_core test_core.cpp)
target_link_libraries(test_core PRIVATE cppplot)
target_compile_definitions(test_core PRIVATE _USE_MATH_DEFINES)
add_test(NAME CoreTests COMMAND test_core)

# Test SVG output
add_executable(test_svg test_svg.cpp)
target_link_libraries(test_svg PRIVATE cppplot)
add_test(NAME SVGTests COMMAND test_svg)
