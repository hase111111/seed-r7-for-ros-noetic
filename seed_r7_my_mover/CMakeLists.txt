cmake_minimum_required(VERSION 3.0.2)
project(seed_r7_my_mover)

# C++17 を使う
add_compile_options(-std=c++17)

# 依存パッケージ
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)

# これを書かないと実行ファイルができない
catkin_package()

# ヘッダファイルの場所を指定，catkin_INCLUDE_DIRS で指定できる
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 一括登録するノードのリスト
set(NODE_LIST
  sample_publisher
  joy2cmd_node
)

# add_executable & target_link_libraries を一括で適用
foreach(node ${NODE_LIST})
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} ${catkin_LIBRARIES})
endforeach()
