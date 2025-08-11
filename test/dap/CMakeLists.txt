cmake_minimum_required(VERSION 3.15)

project(dap_test C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Enable warnings and debug info similar to the original Makefile
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -pedantic -O2 -g)
endif()

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(SRC_DIR ${ROOT_DIR}/src)
set(INCLUDE_DIR ${ROOT_DIR}/include)
set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Math sources from src/math
file(GLOB MATH_SRC
  ${SRC_DIR}/math/*.c
)

# DAP sources from this test directory, excluding the test main
file(GLOB DAP_SRC
  ${TEST_DIR}/*.c
)
list(FILTER DAP_SRC EXCLUDE REGEX "test_DAP\\.c$")

set(TEST_MAIN ${TEST_DIR}/test_DAP.c)

add_executable(dap_test
  ${MATH_SRC}
  ${DAP_SRC}
  ${TEST_MAIN}
)

target_include_directories(dap_test PRIVATE
  ${ROOT_DIR}
  ${INCLUDE_DIR}
  ${TEST_DIR}
)

# Link math library on non-MSVC toolchains
if(NOT MSVC)
  target_link_libraries(dap_test m)
endif()

# Place runtime output in the build directory
set_target_properties(dap_test PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Convenience target to run the test binary
add_custom_target(run
  COMMAND dap_test
  DEPENDS dap_test
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  VERBATIM
)


