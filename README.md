# This repo is an implementation of a simple behavior tree using the BehaviorTreeCpp library. 

The idea is to build a simple "move-stop" robot with behavior tree.

Two ROS topics will be used
- control_mode (std_msgs/Int8) as user command to the robot.
- cmd_vel (geometry_msgs/Twist) as robot control command.

When control_mode value is 21, cmd_vel.linear.x will be 0.4. When control_mode value is 22, cmd_vel.linear.x will be 0.0.
