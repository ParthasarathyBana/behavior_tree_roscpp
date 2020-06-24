cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(behavior_tree_roscpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp std_msgs
    behaviortree_cpp_v3
    actionlib_msgs
    actionlib
    message_generation)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(GTest)

add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
)


add_action_files(
   DIRECTORY action
   FILES     RobotMove.action
   FILES     RobotStop.action
   FILES     ConditionMove.action
   FILES     ConditionStop.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})

######################################################


add_executable(robot_move_action_server "src/action_move_server.cpp")
add_dependencies(robot_move_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_move_action_server ${catkin_LIBRARIES})

add_executable(robot_stop_action_server "src/action_stop_server.cpp")
add_dependencies(robot_stop_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_stop_action_server ${catkin_LIBRARIES})

add_executable(condition_move_action_server "src/condition_move_server.cpp")
add_dependencies(condition_move_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(condition_move_action_server ${catkin_LIBRARIES})

add_executable(condition_stop_action_server "src/condition_stop_server.cpp")
add_dependencies(condition_stop_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(condition_stop_action_server ${catkin_LIBRARIES})

add_executable(test_behavior_tree "src/test_behavior_tree.cpp")
add_dependencies(test_behavior_tree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_behavior_tree ${catkin_LIBRARIES})

