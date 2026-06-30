#!/usr/bin/env python3
import os
import signal
import sys

sys.path.insert(0, '/home/master26/unitree_sdk2_python')

from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize


NETWORK_INTERFACE = 'enx0c3796c78061'
DDS_CMD_TOPIC     = 'rt/arm_Command'


@dataclass
@annotate.final
@annotate.autoid("sequential")
class ArmString_(idl.IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    data_: str = ""


def main():
    ChannelFactoryInitialize(0, NETWORK_INTERFACE)

    pub = ChannelPublisher(DDS_CMD_TOPIC, ArmString_)
    pub.Init()

    # funcode:7 is the firmware's dedicated zero-position command — it uses
    # its own safe trajectory, unlike funcode:2 with angle=0 which can be
    # blocked by firmware soft limits on some joints (e.g. Joint_4).
    pub.Write(ArmString_(data_='{"seq":0,"address":1,"funcode":7}'))
    print('Published funcode:7 (go to zero) command.')
    os._exit(0)


if __name__ == '__main__':
    main()
