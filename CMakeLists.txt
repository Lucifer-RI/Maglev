# 指定最低Cmake版本
cmake_minimum_required(VERSION 3.0.0)

#项目名 
PROJECT(Maglev)

# 指定编译类型
SET(CMAKE_BUILD_TYPE DEBUG)

#指定C++编译版本
SET(CMAKE_CXX_STANDARD 11)

# 指定部分编译选项
# SET(CMAKE_CXX_FLAGS " -lpthread")

# 标记include文件夹为头文件夹路径
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# 搜索所有src目录下的源文件,并命名为SRC
FILE(GLOB SRC ${PROJECT_SOURCE_DIR}/src/*.cc)

# 生成可执行文件Maglev
ADD_EXECUTABLE(${PROJECT_NAME} ${SRC})

#链接库文件
target_link_libraries(${PROJECT_NAME} 
                        # ${PROJECT_SOURCE_DIR}/so
                        -lpthread
                    )      
