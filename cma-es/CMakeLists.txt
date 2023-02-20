cmake_minimum_required(VERSION 2.6)
project(SSC)

add_definitions(
  -O3
  -pedantic
  -Wall
  -Wextra
  -DNDEBUG
)

file(GLOB srcs "*.h")
add_library(cmaes ${srcs})
set_target_properties(cmaes PROPERTIES LINKER_LANGUAGE CXX)
