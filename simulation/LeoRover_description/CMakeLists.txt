cmake_minimum_required(VERSION 3.8)
project(leo_rover_description)

# ===== 基本编译选项（可留可不留，但推荐）=====
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)

# ===== 安装资源到 share 目录，让 ros2 launch 能找到 =====
install(
  DIRECTORY
    launch
    urdf
    meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

