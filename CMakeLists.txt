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
find_package(PCL 1.10 REQUIRED)

# 添加include文件夹
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
)

# 创建库
file(GLOB_RECURSE LIB_SOURCES src/*.cpp)
add_library(pointCloudUtils ${LIB_SOURCES})

# 编译test文件夹中的测试文件
file(GLOB TEST_SOURCES test/*.cpp)
foreach(testsource ${TEST_SOURCES})
    get_filename_component(testname ${testsource} NAME_WE)
    add_executable(${testname} ${testsource})
    target_link_libraries(${testname} ${PCL_LIBRARIES} pointCloudUtils) 
endforeach(testsource ${TEST_SOURCES} )

# 安装库
install(TARGETS pointCloudUtils DESTINATION lib)

# 安装头文件
install(DIRECTORY include/ DESTINATION include)