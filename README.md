# To run

Connect the robot and run:

`ros2 run d1_sim2real d1_bridge_node`

to activate the joint state publisher node.

To check if it's working, open a new terminal and run:

`source /opt/ros/humble/setup.bash`

`ros2 topic echo /d1/joint_states`

To go to the neutral position:

`ros2 topic pub /d1/joint_commands sensor_msgs/JointState "{`
  `header: {stamp: {sec: 0, nanosec: 0}},`
  `name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],`
  `position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],`
  `velocity: [],`
  `effort: []`
`}" -1`


to run the policy:

`python3 src/run_task.py`