cmake_minimum_required(VERSION 3.10)
project(pointCloudUtils)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
# set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "/home/zefu/Projects/BEV-SLAM/third_party/pointCloudUtils" CACHE PATH "Default install path" FORCE)
endif()

# 寻找库
find_package(Eigen3 REQUIRED)
set (BOOST_DIR boost)

option(USE_OPENMP "Enable OpenMP?" ON) # set to OFF to disable
if(USE_OPENMP)
 FIND_PACKAGE(OpenMP)
 if(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP flags = ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 else(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP not supported")
 endif(OPENMP_FOUND OR OpenMP_FOUND)
endif(USE_OPENMP)

# 添加include文件夹
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${BOOST_DIR}
  include
)

# 创建库
file(GLOB_RECURSE LIB_SOURCES src/*.cpp)
add_library(pointCloudUtils SHARED ${LIB_SOURCES})

# 编译test文件夹中的测试文件
file(GLOB TEST_SOURCES test/*.cpp)
foreach(testsource ${TEST_SOURCES})
    get_filename_component(testname ${testsource} NAME_WE)
    add_executable(${testname} ${testsource})
    target_link_libraries(${testname} pointCloudUtils) 
endforeach(testsource ${TEST_SOURCES} )

# 安装库
install(TARGETS pointCloudUtils DESTINATION 
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib) 

# 安装头文件
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/${BOOST_DIR}/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp")