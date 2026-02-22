cmake_minimum_required(VERSION 3.14)

# ─────────────────────────────────────────────────────────────
# Helper: đăng ký một test executable
# ─────────────────────────────────────────────────────────────
function(cppplot_add_test target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE cppplot)
    target_compile_features(${target_name} PRIVATE cxx_std_17)
    # _USE_MATH_DEFINES cần cho MSVC (M_PI, M_E, v.v.)
    target_compile_definitions(${target_name} PRIVATE _USE_MATH_DEFINES)

    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
    )

    add_test(
        NAME    ${target_name}
        COMMAND ${target_name}
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
    )

    set_tests_properties(${target_name} PROPERTIES TIMEOUT 60)
endfunction()

# Tạo thư mục chứa SVG output
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output")

# ─────────────────────────────────────────────────────────────
# NHÓM 1 — Tests GỐC (giữ nguyên tên test để không break gì)
# ─────────────────────────────────────────────────────────────

# test_core — giữ nguyên tên CTest là "CoreTests" như ban đầu
add_executable(test_core test_core.cpp)
target_link_libraries(test_core PRIVATE cppplot)
target_compile_definitions(test_core PRIVATE _USE_MATH_DEFINES)
set_target_properties(test_core PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
)
add_test(NAME CoreTests COMMAND test_core
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
)
set_tests_properties(CoreTests PROPERTIES TIMEOUT 60 LABELS "original")

# test_svg — giữ nguyên tên CTest là "SVGTests" như ban đầu
add_executable(test_svg test_svg.cpp)
target_link_libraries(test_svg PRIVATE cppplot)
target_compile_definitions(test_svg PRIVATE _USE_MATH_DEFINES)
set_target_properties(test_svg PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
)
add_test(NAME SVGTests COMMAND test_svg
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
)
set_tests_properties(SVGTests PROPERTIES TIMEOUT 60 LABELS "original")

# ─────────────────────────────────────────────────────────────
# NHÓM 2 — Tests GỐC chưa được đăng ký (nay đăng ký thêm)
# Các file này đã có trong repo từ đầu nhưng chưa có trong
# CMakeLists gốc → thêm vào để không lãng phí
# ─────────────────────────────────────────────────────────────
cppplot_add_test(test_matrix           test_matrix.cpp)
cppplot_add_test(test_notebook_compat  test_notebook_compat.cpp)
cppplot_add_test(test_sysid            test_sysid.cpp)

set_tests_properties(
    test_matrix test_notebook_compat test_sysid
    PROPERTIES LABELS "original"
)

# ─────────────────────────────────────────────────────────────
# NHÓM 3 — Tests MỚI (bổ sung, kiểm tra sâu hơn)
# ─────────────────────────────────────────────────────────────
cppplot_add_test(test_basic_plot       test_basic_plot.cpp)
cppplot_add_test(test_svg_validity     test_svg_validity.cpp)
cppplot_add_test(test_plot_types       test_plot_types.cpp)
cppplot_add_test(test_control_systems  test_control_systems.cpp)
cppplot_add_test(test_layout           test_layout.cpp)
cppplot_add_test(test_numerical        test_numerical.cpp)

set_tests_properties(
    test_basic_plot test_svg_validity test_plot_types
    test_control_systems test_layout test_numerical
    PROPERTIES LABELS "extended"
)

# ─────────────────────────────────────────────────────────────
# Tóm tắt khi chạy cmake
# ─────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "cppplot test suite:")
message(STATUS "  [original] CoreTests, SVGTests,")
message(STATUS "             test_matrix, test_notebook_compat, test_sysid")
message(STATUS "  [extended] test_basic_plot, test_svg_validity, test_plot_types,")
message(STATUS "             test_control_systems, test_layout, test_numerical")
message(STATUS "")
message(STATUS "  Chay tat ca :  ctest")
message(STATUS "  Chi original:  ctest -L original")
message(STATUS "  Chi extended:  ctest -L extended")
message(STATUS "  Xem chi tiet:  ctest --output-on-failure")
