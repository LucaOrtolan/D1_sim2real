#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ['Joint_0', 'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']
REST_POSITION = [0.0, -1.57, 1.65, 0.0, 0.0, 0.0, 0.0]


class RestPosition(Node):
    def __init__(self):
        super().__init__('rest_position')
        self.pub = self.create_publisher(JointState, '/d1/joint_commands', 10)
        self.timer = self.create_timer(0.1, self._publish_once)

    def _publish_once(self):
        self.timer.cancel()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = REST_POSITION
        self.pub.publish(msg)
        self.get_logger().info('Published rest position command.')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = RestPosition()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
