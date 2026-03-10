# Examples CMakeLists.txt

# Create output directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/output)

# Basic plot example
add_executable(basic_plot basic_plot.cpp)
target_link_libraries(basic_plot PRIVATE cppplot)
target_compile_definitions(basic_plot PRIVATE _USE_MATH_DEFINES)

# Subplots example
add_executable(subplots subplots.cpp)
target_link_libraries(subplots PRIVATE cppplot)
target_compile_definitions(subplots PRIVATE _USE_MATH_DEFINES)

# Scientific example
add_executable(scientific scientific.cpp)
target_link_libraries(scientific PRIVATE cppplot)
target_compile_definitions(scientific PRIVATE _USE_MATH_DEFINES)

# Boxplot labels example
add_executable(boxplot_labels_demo boxplot_labels_demo.cpp)
target_link_libraries(boxplot_labels_demo PRIVATE cppplot)
target_compile_definitions(boxplot_labels_demo PRIVATE _USE_MATH_DEFINES)

# Twin axes example
add_executable(twinx_demo twinx_demo.cpp)
target_link_libraries(twinx_demo PRIVATE cppplot)
target_compile_definitions(twinx_demo PRIVATE _USE_MATH_DEFINES)

# Contour example
add_executable(contour_demo contour_demo.cpp)
target_link_libraries(contour_demo PRIVATE cppplot)
target_compile_definitions(contour_demo PRIVATE _USE_MATH_DEFINES)

# Polar example
add_executable(polar_demo polar_demo.cpp)
target_link_libraries(polar_demo PRIVATE cppplot)
target_compile_definitions(polar_demo PRIVATE _USE_MATH_DEFINES)

# Copy output directory structure
add_custom_target(create_output_dir ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/output
)
