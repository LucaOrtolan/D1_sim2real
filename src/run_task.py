import math

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from d1_wrapper import D1ReachPolicy


class ReachPolicy(Node):
    """ROS2 node for controlling a Gen3 robot's reach policy."""
    
    # Define simulation degree-of-freedom angle limits: (Lower limit, Upper limit, Inversed flag)
    SIM_DOF_ANGLE_LIMITS = [
        (-360, 360, False),
        (-360, 360, False),
        (-360, 360, False),
        (-360, 360, False),
        (-360, 360, False),
        (-360, 360, False),
    ]
    
    # Define servo angle limits (in radians)
    PI = math.pi
    SERVO_ANGLE_LIMITS = [
        (-3/4 * PI, 3/4 * PI),
        (-PI/2, PI/2),
        (-PI/2, PI/2),
        (-3/4 * PI, 3/4 * PI),
        (-PI/2, PI/2),
        (-3/4 * PI, 3/4 * PI),
    ]
    
    # ROS topics — bridge node publishes states here and listens for commands here
    STATE_TOPIC = '/d1/joint_states'
    CMD_TOPIC   = '/d1/joint_commands'

    # Names must match the IsaacLab training env joint names exactly
    JOINT_NAMES = [
        'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6',
        'Joint_L', 'Joint_R',
    ]

    JOINT_NAME_TO_IDX = {
        'Joint1': 0, 'Joint2': 1, 'Joint3': 2, 'Joint4': 3,
        'Joint5': 4, 'Joint6': 5, 'Joint_L': 6, 'Joint_R': 7,
    }

    def __init__(self, fail_quietly: bool = False, verbose: bool = False):
        """Initialize the ReachPolicy node."""
        super().__init__('reach_policy_node')
        
        self.robot = D1ReachPolicy()
        self.target_command = np.zeros(7)
        self.step_size = 1.0 / 100  # 10 ms period = 100 Hz
        self.timer = self.create_timer(self.step_size, self.step_callback)
        self.i = 0
        self.fail_quietly = fail_quietly
        self.verbose = verbose
        # self.pub_freq = 1.0  # Hz
        self.current_pos = None  # Dictionary of current joint positions
        self.target_pos = None   # List of target joint positions

        # Subscriber for joint state feedback from the D1 bridge
        self.create_subscription(JointState, self.STATE_TOPIC, self.sub_callback, 10)

        # Publisher for joint position commands to the D1 bridge
        self.pub = self.create_publisher(JointState, self.CMD_TOPIC, 10)
        
        self.get_logger().info("ReachPolicy node initialized.")

    def sub_callback(self, msg: JointState):
        """Callback for joint state feedback from the D1 bridge."""
        self.current_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
        self.robot.update_joint_state(msg.position, msg.velocity)

    def map_joint_angle(self, pos: float, index: int) -> float:
        """
        Map a simulation joint angle (in radians) to the real-world servo angle (in radians).
        
        Args:
            pos: Joint angle from simulation (in radians).
            index: Index of the joint.
        
        Returns:
            Mapped joint angle within the servo limits.
        """
        L, U, inversed = self.SIM_DOF_ANGLE_LIMITS[index]
        A, B = self.SERVO_ANGLE_LIMITS[index]
        angle_deg = np.rad2deg(float(pos))
        # Check if the simulation angle is within limits
        if not L <= angle_deg <= U:
            self.get_logger().warn(
                f"Simulation joint {index} angle ({angle_deg}) out of range [{L}, {U}]. Clipping."
            )
            angle_deg = np.clip(angle_deg, L, U)
        # Map the angle from the simulation range to the servo range
        mapped = (angle_deg - L) * ((B - A) / (U - L)) + A
        if inversed:
            mapped = (B - A) - (mapped - A) + A
        # Verify the mapped angle is within servo limits
        if not A <= mapped <= B:
            raise Exception(
                f"Mapped joint {index} angle ({mapped}) out of servo range [{A}, {B}]."
            )
        return mapped

    def step_callback(self):
        """
        Timer callback to compute and publish the next joint trajectory command.
        """
        # Set a constant target command for the robot (example values)
        if self.i%3000 < 1000:
            self.target_command = np.array([0.5, 0.0, 0.2, 0.7071, 0.0, 0.7071, 0.0])
        elif self.i%3000 < 2000 and self.i%3000 > 1000:
            self.target_command = np.array([0.4, -0.15, 0.3, 0.7071, 0.0, 0.7071, 0.0])
        else:
            self.target_command = np.array([0.6, 0.1, 0.45, 0.7071, 0.0, 0.7071, 0.0])

        # self.target_command = np.array([0.3, 0.0, 0.35, 0.0, 0.0, 0.0, 0.0])

        # Get simulation joint positions from the robot's forward model
        joint_pos = self.robot.forward(self.step_size, self.target_command)
        
        if joint_pos is not None:
            if len(joint_pos) != 8:
                raise Exception(f"Expected 8 joint positions, got {len(joint_pos)}!")

            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = self.JOINT_NAMES
            cmd.position = joint_pos.tolist()
            self.pub.publish(cmd)
            
        self.i += 1
        

def main(args=None):
    rclpy.init(args=args)
    node = ReachPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()