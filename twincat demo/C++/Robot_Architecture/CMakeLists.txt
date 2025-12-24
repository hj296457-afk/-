#minimum version of cmake
cmake_minimum_required(VERSION 3.5)

# 工程名称、语言、版本
project(Robot
  LANGUAGES C CXX
  VERSION 0.1.0
  )

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g")



set(target_name start_robot)

file(GLOB src_file_     ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB include_file_ ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB hpp_file_ ${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/*.hpp)

list(APPEND dir
     Control
     EcMaster
     Robot


)
foreach(_dir ${dir})
file(GLOB src_file     ${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/*.cpp)
file(GLOB src_file_c     ${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/*.c)
file(GLOB include_file ${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/*.h)
file(GLOB hpp_file ${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/*.hpp)
list(APPEND src
    ${src_file_}
    ${src_file_c}
    ${include_file_}
    ${hpp_file_}
    ${src_file}
    ${include_file}
    ${hpp_file}
  )
endforeach()

add_executable(${target_name} "")

target_sources(${target_name}
  PRIVATE
    ${src}
  )

target_link_libraries(${target_name}
  PUBLIC
      pthread

)
target_compile_options(${target_name}
  PRIVATE
    -std=c++14
  )
