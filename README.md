# To run

Connect the robot and run:

`ros2 run d1_sim2real d1_bridge_node`

to activate the joint state publisher node.

To check if it's working, open a new terminal and run:

`source /opt/ros/humble/setup.bash`

`ros2 topic echo /d1/joint_states`

To go to the neutral position:

`sudo /home/master26/Documents/d1_sdk/build/arm_zero_control`

to run the policy:

`python3 src/run_task.py`