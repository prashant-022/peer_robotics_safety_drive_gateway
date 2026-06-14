# peer_robotics_safety_drive_gateway
A ROS 2 safety arbitration node that sits between an autonomous navigation stack and the drive hardware.  The node receives planner velocity commands, applies safety rules, and publishes the final velocity command that reaches the motors.
