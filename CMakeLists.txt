cmake_minimum_required(VERSION 3.18)

project(betaflight_control_core C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(bfcore
    src/control_core.c
)

target_include_directories(bfcore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(bfcore PRIVATE
    $<$<C_COMPILER_ID:Clang,GNU>:-Wall -Wextra -Wpedantic>
)

add_executable(bfcore_step_demo
    examples/step_demo.c
)

target_link_libraries(bfcore_step_demo PRIVATE bfcore)

enable_testing()

add_executable(bfcore_tests
    tests/test_control_core.c
)

target_link_libraries(bfcore_tests PRIVATE bfcore)

add_test(NAME bfcore_tests COMMAND bfcore_tests)
