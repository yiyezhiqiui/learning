cmake_minimum_required(VERSION 3.2)
cmake_policy(SET CMP0057 NEW)


project(picture)

file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

set(CMAKE_PREFIX_PATH "/usr/local/share/behaviortree_cpp/cmake")


find_package(behaviortree_cpp REQUIRED)

include_directories(${behaviortree_cpp_INCLUDE_DIRS})
add_executable(test ${SRC})
target_link_libraries(test ${behaviortree_cpp_LIBRARIES})

