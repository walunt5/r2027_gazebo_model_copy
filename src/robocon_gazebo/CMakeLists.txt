cmake_minimum_required(VERSION 3.8)
project(robocon_gazebo)

find_package(ament_cmake REQUIRED)

# 从仓库克隆目录 (workspace/tool/gazebo_models) 拷贝场地资源到 install/share。
# 若仓库位置变动, 用 -DROBOCON_MODELS_DIR=/path/to/gazebo_models 覆盖。
set(ROBOCON_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../tool/gazebo_models"
    CACHE PATH "Path to the gazebo_models clone")

if(NOT EXISTS "${ROBOCON_MODELS_DIR}/robocon_track.world")
  message(FATAL_ERROR
    "找不到 ${ROBOCON_MODELS_DIR}/robocon_track.world, 请设置 -DROBOCON_MODELS_DIR=<gazebo_models 路径>")
endif()

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/tower_top_placement_gui.py scripts/tower_base_placement_gui.py
  DESTINATION lib/${PROJECT_NAME})

# 包自带的完整碰撞模型 (L1/L2 实体 + 坡道 + 阶梯)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

install(FILES "${ROBOCON_MODELS_DIR}/robocon_track.world"
  DESTINATION share/${PROJECT_NAME}/worlds)

# 整目录安装 Gazebo 模型 (model.config + model.sdf + meshes)
install(DIRECTORY "${ROBOCON_MODELS_DIR}/robocon_ground"
  DESTINATION share/${PROJECT_NAME}/models
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.py" EXCLUDE
  PATTERN "mujoco" EXCLUDE)

ament_package()
