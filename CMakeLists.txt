cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 23)

# Define the project name
project(t CXX)

# Define the executable name and source file
add_executable(${PROJECT_NAME} 
main.cpp
serialController.cpp
)

# Link the pthreads library (necessary for C++ <thread>)
target_link_libraries(${PROJECT_NAME} PRIVATE pthread)

# Configuration for static compilation for embedded/cross-platform deployment.
# These flags are essential to avoid the "required file not found" error
# on the BTT Pi by embedding all C and C++ runtime libraries.
target_compile_options(${PROJECT_NAME} PRIVATE -static -static-libgcc -static-libstdc++)