cmake_minimum_required(VERSION 3.10)
project(cpp_eign)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(eigen eigen.cpp)
target_link_libraries(eigen Eigen3::Eigen)
