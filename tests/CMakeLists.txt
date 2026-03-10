cmake_minimum_required(VERSION 3.14)

# ─────────────────────────────────────────────────────────────
# Helper: đăng ký một test executable
# ─────────────────────────────────────────────────────────────
# ── Catch2 via FetchContent ──────────────────────────────────
include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Catch2)

# Fix MinGW: gmtime_s không tồn tại trong MinGW runtime → patch thành gmtime_r
if(MINGW)
    set(_junit "${catch2_SOURCE_DIR}/src/catch2/reporters/catch_reporter_junit.cpp")
    if(EXISTS "${_junit}")
        file(READ "${_junit}" _src)
        string(FIND "${_src}" "gmtime_s(&timeInfo, &rawtime)" _pos)
        if(NOT _pos EQUAL -1)
            string(REPLACE
                "gmtime_s(&timeInfo, &rawtime)"
                "gmtime_r(&rawtime, &timeInfo)"
                _src "${_src}")
            file(WRITE "${_junit}" "${_src}")
            message(STATUS "Catch2: patched gmtime_s -> gmtime_r (MinGW)")
        endif()
    endif()
endif()


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
cppplot_add_test(test_fixes            test_fixes.cpp)

set_tests_properties(
    test_basic_plot test_svg_validity test_plot_types
    test_control_systems test_layout test_numerical test_fixes
    PROPERTIES LABELS "extended"
)

# ── QP Solver test suite — M1 milestone ─────────────────────
add_executable(test_qp_solver test_qp_solver.cpp)

target_include_directories(test_qp_solver PRIVATE
    "${CMAKE_SOURCE_DIR}/include"   # → include/cppplot/control/qp_solver.hpp
)

target_link_libraries(test_qp_solver PRIVATE
    cppplot          # provides qp_solver.hpp include path
    Catch2::Catch2WithMain
)

target_compile_features(test_qp_solver PRIVATE cxx_std_17)
target_compile_definitions(test_qp_solver PRIVATE _USE_MATH_DEFINES)

set_target_properties(test_qp_solver PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/output"
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
