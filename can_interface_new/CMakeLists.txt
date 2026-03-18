cmake_minimum_required(VERSION 3.10)
project(can_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add your executable
add_executable(can_interface src/main.cpp src/can_interface.cpp)

target_include_directories(can_interface PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Link required libraries (if needed)
# target_link_libraries(can_interface pthread)