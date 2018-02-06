import argparse
import math

import functools
import rospy
from std_msgs.msg import *
import time

from pypmd import PMD
import logging


def rearrange_pot_read(analog_read):
    return [analog_read[7], analog_read[1], analog_read[6], analog_read[3]]


class IOPMD:
    def __init__(self, arm='MTML', pmd1=None, pmd5=None, **kwargs):
        self.arm = arm
        self.ros_io_namespace = f'/dvrk/{self.arm}/io/external'

        self.pub_encoder_position = rospy.Publisher(f'{self.ros_io_namespace}/encoder_position',
                                                    std_msgs.msg.Int32MultiArray, queue_size=1)
        self.pub_encoder_velocity = rospy.Publisher(f'{self.ros_io_namespace}/encoder_velocity',
                                                    std_msgs.msg.Int32MultiArray, queue_size=1)
        self.pub_current_feedback = rospy.Publisher(f'{self.ros_io_namespace}/current_feedback',
                                                    std_msgs.msg.Float64MultiArray, queue_size=1)
        self.pub_pot_voltage = rospy.Publisher(f'{self.ros_io_namespace}/pot_voltage',
                                               std_msgs.msg.Float64MultiArray, queue_size=1)
        self.sub_requested_current = rospy.Subscriber(f'{self.ros_io_namespace}/requested_current',
                                                      std_msgs.msg.Float64MultiArray, self.set_current_cb)
        self.sub_encoder_position_set = rospy.Subscriber(f'{self.ros_io_namespace}/encoder_position_set',
                                                         std_msgs.msg.Int32MultiArray, self.set_encoder_cb)

        self.requested_current = [0.] * 8

        pmd_1 = PMD(host=pmd1)
        pmd_5 = PMD(host=pmd5)

        # pmd_1.parse_script('../config/c_mo_2.txt')
        # pmd_1.set_operating_mode(1, axis_enabled=1, motor_output_enabled=1, current_control_enabled=1)

        self.pmd = [pmd_1, pmd_5]
        [x.multi_update() for x in self.pmd]


    def sync_states(self):
        pots = [(4.5 / 5) * x for x in
                (rearrange_pot_read(self.pmd[0].read_analogs()) + rearrange_pot_read(self.pmd[1].read_analogs()))]
        self.pub_pot_voltage.publish(std_msgs.msg.Float64MultiArray(data=pots))

        encoder_positions = [self.pmd[0].read_encoder_position(i) for i in range(4)] + [
            self.pmd[1].read_encoder_position(i) for i in range(4)]
        self.pub_encoder_position.publish(std_msgs.msg.Int32MultiArray(data=encoder_positions))

        encoder_velocities = [self.pmd[0].read_encoder_velocity(i) for i in range(4)] + [
            self.pmd[1].read_encoder_velocity(i) for i in range(4)]
        self.pub_encoder_velocity.publish(std_msgs.msg.Int32MultiArray(data=encoder_velocities))

        motor_currents = [self.pmd[0].read_motor_current(i) for i in range(4)] + [self.pmd[1].read_motor_current(i)
                                                                                  for i in range(4)]
        self.pub_current_feedback.publish(std_msgs.msg.Float64MultiArray(data=motor_currents))

        [self.pmd[0].set_motor_current(axis, current, full_scale_current=4) for axis, current in enumerate(self.requested_current[:4])]
        [self.pmd[1].set_motor_current(axis, current) for axis, current in enumerate(self.requested_current[4:])]

        [x.multi_update() for x in self.pmd]

    def set_current_cb(self, data):
        self.requested_current = data.data

    def set_encoder_cb(self, data):
        print('set_encoder_cb')
        print(data)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', help='name of the arm')
    parser.add_argument('--pmd1', help='hostname of the pmd for joint 1-4')
    parser.add_argument('--pmd5', help='hostname of the pmd for joint 5-8')
    args = parser.parse_args()

    rospy.init_node('dvrk_io_pmd')
    print('node started')

    io_pmd = IOPMD(arm=args.arm, pmd1=args.pmd1, pmd5=args.pmd5)
    print('pmd init')

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        io_pmd.sync_states()
        # r.sleep()
        # rospy.spin()
