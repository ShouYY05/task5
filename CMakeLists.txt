# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(task5)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target built from your sources  
# 这里括号里面的两个参数分别是工程项目名和我们要编译文件名的意思，记住中间一空格键隔开
add_executable(task5 main.cpp)

# Link your application with OpenCV libraries
target_link_libraries(task5 PRIVATE ${OpenCV_LIBS})
