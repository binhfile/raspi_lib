cmake_minimum_required(VERSION 3.15)
project(micro_http C)

set(CMAKE_C_STANDARD 99)

include_directories(.)

file(GLOB_RECURSE SRC_GLOB_LIST *.c)
list(APPEND SRC_LIST ${SRC_GLOB_LIST})

add_executable(ds18b20 ${SRC_LIST})
