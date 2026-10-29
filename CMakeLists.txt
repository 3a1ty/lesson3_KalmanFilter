cmake_minimum_required(VERSION 3.16)
project(ekf_armor LANGUAGES CXX)

# ---------------------------------------------------------------------------
# 编译选项
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra)

# ---------------------------------------------------------------------------
# Eigen：优先用系统安装的，找不到就退回 third_party/eigen-3.4.0（头文件库）
# ---------------------------------------------------------------------------
find_package(Eigen3 3.3 QUIET NO_MODULE)

if(TARGET Eigen3::Eigen)
  message(STATUS "Eigen: 使用系统安装版本")
else()
  set(EIGEN_LOCAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.4.0")
  if(EXISTS "${EIGEN_LOCAL_DIR}/Eigen/Dense")
    add_library(eigen3_local INTERFACE)
    target_include_directories(eigen3_local INTERFACE "${EIGEN_LOCAL_DIR}")
    add_library(Eigen3::Eigen ALIAS eigen3_local)
    message(STATUS "Eigen: 使用 ${EIGEN_LOCAL_DIR}")
  else()
    message(FATAL_ERROR
      "找不到 Eigen，二选一：\n"
      "  1) sudo apt install libeigen3-dev\n"
      "  2) 把 Eigen 解压到 third_party/eigen-3.4.0，"
      "使 third_party/eigen-3.4.0/Eigen/Dense 存在")
  endif()
endif()

# ---------------------------------------------------------------------------
# 数据文件：work/ 里只放代码，观测数据留在上一层
# ---------------------------------------------------------------------------
set(DATA_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../armor_data.csv")
if(NOT EXISTS "${DATA_FILE}")
  set(DATA_FILE "${CMAKE_CURRENT_SOURCE_DIR}/armor_data.csv")
endif()
if(NOT EXISTS "${DATA_FILE}")
  message(FATAL_ERROR
    "找不到 armor_data.csv，应在 ${CMAKE_CURRENT_SOURCE_DIR}/.. 或 ${CMAKE_CURRENT_SOURCE_DIR} 下")
endif()
message(STATUS "数据文件: ${DATA_FILE}")

# ---------------------------------------------------------------------------
# 可执行文件
# ---------------------------------------------------------------------------
add_executable(ekf_armor
  main.cpp
  csv_reader.cpp
  kalman_filter.cpp
)

target_include_directories(ekf_armor PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(ekf_armor PRIVATE Eigen3::Eigen)

# 把数据拷一份到构建目录，程序里可以直接写 "armor_data.csv"
add_custom_command(TARGET ekf_armor POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${DATA_FILE}"
          "$<TARGET_FILE_DIR:ekf_armor>/armor_data.csv"
  COMMENT "拷贝 armor_data.csv 到构建目录")
