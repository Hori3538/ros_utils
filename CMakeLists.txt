cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++17)
project(ros_utils)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ros_utils
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(odom_topic_to_tf_node src/odom_topic_to_tf_node.cpp src/odom_topic_to_tf.cpp)
add_dependencies(odom_topic_to_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    odom_topic_to_tf_node
    ${catkin_LIBRARIES}
    )

add_executable(publish_odom_as_pose_node src/publish_odom_as_pose_node.cpp src/publish_odom_as_pose.cpp)
add_dependencies(publish_odom_as_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    publish_odom_as_pose_node
    ${catkin_LIBRARIES}
    )

add_executable(odom_transformer_node src/odom_transformer_node.cpp src/odom_transformer.cpp)
add_dependencies(odom_transformer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    odom_transformer_node
    ${catkin_LIBRARIES}
    )
