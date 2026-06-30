#!/usr/bin/env python3
# Requires: pip install cyclonedds
# Requires: unitree_sdk2_python in PYTHONPATH or at the path below
import math
import os
import signal
import sys
import threading

sys.path.insert(0, '/home/master26/unitree_sdk2_python')

from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize,
)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


NETWORK_INTERFACE = 'enx0c3796c78061'
DDS_SERVO_TOPIC   = 'current_servo_angle'
DDS_CMD_TOPIC     = 'rt/arm_Command'
ROS_STATE_TOPIC   = '/d1/joint_states'
ROS_CMD_TOPIC     = '/d1/joint_commands'
JOINT_NAMES = [
    'Joint_0', 'Joint_1', 'Joint_2', 'Joint_3',
    'Joint_4', 'Joint_5', 'Joint_6',
]


@dataclass
@annotate.final
@annotate.autoid("sequential")
class ArmString_(idl.IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    data_: str = ""


@dataclass
@annotate.final
@annotate.autoid("sequential")
class PubServoInfo_(idl.IdlStruct, typename="unitree_arm::msg::dds_::PubServoInfo_"):
    servo0_data_: types.float32 = 0.0
    servo1_data_: types.float32 = 0.0
    servo2_data_: types.float32 = 0.0
    servo3_data_: types.float32 = 0.0
    servo4_data_: types.float32 = 0.0
    servo5_data_: types.float32 = 0.0
    servo6_data_: types.float32 = 0.0


class D1BridgeNode(Node):
    def __init__(self):
        super().__init__('d1_bridge_node')
        self._seq = 0
        self._lock = threading.Lock()
        self._has_prev = False
        self._prev_pos_deg = [0.0] * 7
        self._prev_time_sec = 0.0

        self._state_pub = self.create_publisher(JointState, ROS_STATE_TOPIC, 10)
        self.create_subscription(JointState, ROS_CMD_TOPIC, self._on_joint_command, 10)

        # DDS publisher to the arm firmware
        self._arm_pub = ChannelPublisher(DDS_CMD_TOPIC, ArmString_)
        self._arm_pub.Init()

        # DDS subscriber for servo feedback — callback runs in a CycloneDDS thread
        self._servo_sub = ChannelSubscriber(DDS_SERVO_TOPIC, PubServoInfo_)
        self._servo_sub.Init(self._on_servo_feedback)

        self._feedback_sub = ChannelSubscriber('arm_Feedback', ArmString_)
        self._feedback_sub.Init(lambda msg: self.get_logger().info(f'arm_Feedback: {msg.data_}'))

        self.get_logger().info(f"D1 bridge ready. Listening on DDS '{DDS_SERVO_TOPIC}'.")

    def _on_servo_feedback(self, pm: PubServoInfo_):
        pos_deg = [
            pm.servo0_data_, pm.servo1_data_, pm.servo2_data_,
            pm.servo3_data_, pm.servo4_data_, pm.servo5_data_,
            pm.servo6_data_,
        ]

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        vel_rads = [0.0] * 7

        with self._lock:
            if self._has_prev:
                dt = now_sec - self._prev_time_sec
                if dt > 1e-6:
                    for i in range(7):
                        vel_rads[i] = (pos_deg[i] - self._prev_pos_deg[i]) * math.pi / 180.0 / dt
            self._prev_pos_deg = pos_deg[:]
            self._prev_time_sec = now_sec
            self._has_prev = True

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [p * math.pi / 180.0 for p in pos_deg]
        msg.velocity = vel_rads
        self._state_pub.publish(msg)

    def _on_joint_command(self, msg: JointState):
        if len(msg.position) < 7:
            self.get_logger().warn(
                f'Joint command has {len(msg.position)} positions, expected 7 — ignoring.'
            )
            return

        def deg(rad): return rad * 180.0 / math.pi

        seq = self._seq
        self._seq += 1

        json_str = (
            f'{{"seq":{seq},"address":1,"funcode":2,"data":{{'
            f'"mode":1'
            f',"angle0":{deg(msg.position[0]):.3f}'
            f',"angle1":{deg(msg.position[1]):.3f}'
            f',"angle2":{deg(msg.position[2]):.3f}'
            f',"angle3":{deg(msg.position[3]):.3f}'
            f',"angle4":{deg(msg.position[4]):.3f}'
            f',"angle5":{deg(msg.position[5]):.3f}'
            f',"angle6":{deg(msg.position[6]):.3f}'
            f'}}}}'
        )
        self._arm_pub.Write(ArmString_(data_=json_str))


def main(args=None):
    # Must initialize Unitree DDS before rclpy.init() — both use CycloneDDS and
    # ChannelFactory.Init() is not idempotent once rclpy already owns the domain.
    ChannelFactoryInitialize(0, NETWORK_INTERFACE)

    rclpy.init(args=args)

    # Create the node (and all DDS entities) before setting signal handlers —
    # CycloneDDS reinstalls its own C-level SIGINT handler during DataWriter/DataReader
    # creation and would override any handler set before this point.
    node = D1BridgeNode()

    signal.signal(signal.SIGINT,  lambda *_: os._exit(0))
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    os._exit(0)


if __name__ == '__main__':
    main()
