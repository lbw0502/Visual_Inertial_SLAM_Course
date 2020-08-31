cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
project(update)

find_package(Sophus REQUIRED)

include_directories("/usr/local/include/eigen3")

add_executable(update update.cpp)
target_link_libraries(update Sophus::Sophus)



