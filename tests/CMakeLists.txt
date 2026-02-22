cmake_minimum_required(VERSION 3.14)

# ─────────────────────────────────────────────
# Helper: thêm một test executable
# ─────────────────────────────────────────────
function(cppplot_add_test target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE cppplot)
    target_compile_features(${target_name} PRIVATE cxx_std_17)
    
    # Chạy trong thư mục build/tests để SVG output tập trung một chỗ
    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
    )
    
    add_test(
        NAME ${target_name}
        COMMAND ${target_name}
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
    )
    
    # Timeout 30 giây mỗi test
    set_tests_properties(${target_name} PROPERTIES TIMEOUT 30)
endfunction()

# ─────────────────────────────────────────────
# Tạo thư mục output cho SVG files
# ─────────────────────────────────────────────
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output")

# ─────────────────────────────────────────────
# Đăng ký các test cases
# ─────────────────────────────────────────────
cppplot_add_test(test_basic_plot       test_basic_plot.cpp)
cppplot_add_test(test_svg_validity     test_svg_validity.cpp)
cppplot_add_test(test_plot_types       test_plot_types.cpp)
cppplot_add_test(test_control_systems  test_control_systems.cpp)
cppplot_add_test(test_layout           test_layout.cpp)
cppplot_add_test(test_numerical        test_numerical.cpp)

# Labels để chạy từng nhóm riêng
set_tests_properties(test_basic_plot test_svg_validity
    PROPERTIES LABELS "core")
set_tests_properties(test_plot_types test_layout
    PROPERTIES LABELS "visual")
set_tests_properties(test_control_systems test_numerical
    PROPERTIES LABELS "controls")
