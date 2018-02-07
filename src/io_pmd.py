import argparse
import threading

import rospy
import logging

from pypmd import PMD
from std_msgs.msg import *

import custom_protocol
from custom_protocol import PMDCustomProtocol

current_direction = (-1, 1, 1, 1, 1, 1, 1, 1)


def rearrange_pot_read(analog_read):
    return [analog_read[7], analog_read[1], analog_read[6], analog_read[3]]


class IOPMD:
    def __init__(self, arm='MTML', pmd1=None, pmd5=None, **kwargs):
        self.arm = arm

        self.requested_current = [0.] * 8
        self.buffered_states = {
            'mode': [0.] * 8,
            'current': [0.] * 8,
            'velocity': [0.] * 8,
            'position': [0.] * 8,
            'temperature': [0.] * 8,
            'analog': [0.] * 8,
            'fault': [0.] * 8,
        }

        self.buffered_commands = {
            'command': 0,
            'command_payload': [0.] * 8,
            'mode': [0] * 8,
            'motor_command': [0] * 8,
        }

        self.publish_sem = {0: threading.BoundedSemaphore(1), 4: threading.BoundedSemaphore(1)}

        self.pmd_rp = (PMD(host=pmd1), PMD(host=pmd5))





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

        threading.Thread(target=self.publish_states_loop, daemon=True).start()

        pmd_1 = PMDCustomProtocol(ip_address=pmd1, offset=0)
        pmd_1.start_receive_loop(self.pmd_callback)
        pmd_5 = PMDCustomProtocol(ip_address=pmd5, offset=4)
        pmd_5.start_receive_loop(self.pmd_callback)

        self.pmd_custom = (pmd_1, pmd_5)


        logging.info('pmd init complete')

    def pmd_callback(self, offset, **kwargs):
        try:
            self.publish_sem[offset].release()
        except ValueError:
            pass

        for k, v in kwargs.items():
            self.buffered_states[k][offset:offset + 4] = v

    def publish_states(self):
        pots = [(4.5 / 5) * (10 / 32767) * x for x in self.buffered_states['analog']]
        self.pub_pot_voltage.publish(std_msgs.msg.Float64MultiArray(data=pots))

        encoder_positions = self.buffered_states['position']
        self.pub_encoder_position.publish(std_msgs.msg.Int32MultiArray(data=encoder_positions))

        encoder_velocities = self.buffered_states['velocity']
        self.pub_encoder_velocity.publish(std_msgs.msg.Int32MultiArray(data=encoder_velocities))

        motor_currents = [x / custom_protocol.AMPS_TO_BITS for x in self.buffered_states['current']]
        motor_currents[7] = 0  # todo: fix me
        self.pub_current_feedback.publish(std_msgs.msg.Float64MultiArray(data=motor_currents))

        print(f'\r{self.buffered_states}', end='')
        sys.stdout.flush()

    def publish_states_loop(self):
        while True:
            [sem.acquire() for sem in self.publish_sem.values()]
            self.publish_states()

    def set_current_cb(self, data):
        requested_current = data.data
        self.pmd_custom[0].send(mode=(custom_protocol.MODES['current'],) * 4, motor_command=[
            int(custom_protocol.AMPS_TO_BITS * a) * 2 * current_direction[i] for i, a in enumerate(requested_current[0:4])])
        self.pmd_custom[1].send(mode=(custom_protocol.MODES['current'],) * 4,
                                motor_command=[int(custom_protocol.AMPS_TO_BITS * a) * current_direction[i + 4] for i, a
                                               in enumerate(requested_current[4:8])])

    def set_encoder_cb(self, data):
        positions = data.data
        self.pmd_rp[0].set_actual_encoder_position(positions[0:4])
        self.pmd_rp[1].set_actual_encoder_position(positions[4:8])
        print(f'set encoder position to {positions}')


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', help='name of the arm')
    parser.add_argument('--pmd1', help='hostname of the pmd for joint 1-4')
    parser.add_argument('--pmd5', help='hostname of the pmd for joint 5-8')
    args = parser.parse_args()

    rospy.init_node('dvrk_io_pmd')
    print('node started')

    io_pmd = IOPMD(arm=args.arm, pmd1=args.pmd1, pmd5=args.pmd5)
    print('pmd init')


    while not rospy.is_shutdown():
        # io_pmd.sync_states()
        # r.sleep()
        rospy.spin()
