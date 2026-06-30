# To run

Connect the robot and run:

`ros2 run d1_sim2real d1_bridge_node`

to activate the joint state publisher node.

To check if it's working, open a new terminal and run:

`source /opt/ros/humble/setup.bash`

`ros2 topic echo /d1/joint_states`

To go to the neutral position:

`ros2 launch d1_sim2real arm_zero_control.launch.py`

to run the policy:

`python3 src/run_task.py`

To teleoperate each joint:

`ros2 topic pub /d1/joint_commands sensor_msgs/msg/JointState "{ header: {stamp: {sec: 0, nanosec: 0}}, name: [Joint_0, Joint_1, Joint_2, Joint_3, Joint_4, Joint_5, Joint_6], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [], effort: [] }" -1`
