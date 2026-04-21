# cmake_minimum_required(VERSION 3.10)
# project(FrankaControl)

# # 设置 C++ 标准
# set(CMAKE_CXX_STANDARD 17)

# # 定义 MuJoCo 路径（指向你之前安装的位置）
# # set(MUJOCO_DIR "$ENV{HOME}/tools/mujoco")
# find_package(Eigen3 REQUIRED)


# add_library(robot_utils STATIC src/utils.cpp)

# target_include_directories(robot_utils PUBLIC include)
# # 1. 添加头文件搜索路径
# # include_directories(${MUJOCO_DIR}/include)
# target_link_libraries(robot_utils PUBLIC Eigen3::Eigen)
# # include_directories(${EIGEN3_INCLUDE_DIRS}) 
# # 2. 添加库文件搜索路径
# # link_directories(${MUJOCO_DIR}/lib)

# # 3. 创建可执行文件
# # add_executable(ik src/ik.cpp)
# # add_executable(rk_my src/eigen.cpp)

# # 4. 链接 MuJoCo 库
# # target_link_libraries(get_angle mujoco)

# # 5. 设置运行时库路径（防止运行报错）
# # set_target_properties(get_angle PROPERTIES INSTALL_RPATH "${MUJOCO_DIR}/lib")
# # set_target_properties(get_angle PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)


cmake_minimum_required(VERSION 3.12)
project(myrobot)


#进行这个库的编译 utils

# find_package(Eigen3 REQUIRED)

# add_library(utils STATIC src/utils.cpp)

# target_link_libraries(utils PUBLIC Eigen3::Eigen)



# #编译这个arm类
# find_package(Eigen3 REQUIRED)

# # 寻找 Eigen3
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# # --- 2. 静态库目标 (Provider) ---
# # 编译库的源文件
# add_library(utils STATIC src/utils.cpp)

# # 绑定依赖与头文件路径
# target_link_libraries(utils PUBLIC Eigen3::Eigen)
# target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# # --- 3. 可执行文件目标 (Consumer) ---
# # 定义程序入口文件
# add_executable(arm src/arm.cpp)

# # 核心：将可执行文件与库进行逻辑链接
# # 此时 pose_app 会自动继承 pose_utils 的 Eigen 路径和头文件声明
# target_link_libraries(arm PRIVATE utils)



# #编译这个arm类
# find_package(Eigen3 REQUIRED)

# # 寻找 Eigen3
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# # --- 2. 静态库目标 (Provider) ---
# # 编译库的源文件
# add_library(my_arm STATIC src/utils.cpp src/arm.cpp)

# # 绑定依赖与头文件路径
# target_link_libraries(my_arm PUBLIC Eigen3::Eigen)
# target_include_directories(my_arm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# # --- 3. 可执行文件目标 (Consumer) ---
# # 定义程序入口文件
# # add_executable(my_arm src/arm.cpp)




# 1. 开启极致性能模式
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
# -O3: 最高级优化; -march=native: 压榨你拯救者 CPU 的每一分性能
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")

# 2. 寻找 Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 3. 编译 pybind11 (确保你 extern 目录下有这个文件夹)
add_subdirectory(extern/pybind11)

# 4. 定义 Python 模块 (这里只准出现一次！)
pybind11_add_module(myrobot_py
    src/my_arm_bind.cpp 
    src/arm_bind.cpp 
    src/utils.cpp 
)

# 5. 绑定依赖
target_link_libraries(myrobot_py PUBLIC 
    Eigen3::Eigen
    pybind11::module
)

# 6. 包含头文件路径
target_include_directories(myrobot_py PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)