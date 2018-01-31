import rospy
import sensor_msgs

sim_suffix = '_SIM'
from pypmd import PMD


class IOPMD:
    def __init__(self, axis_offset=0, arm='MTML', **kwargs):
        self.axis_offset = axis_offset
        self.arm = arm

        rospy.init_node('dvrk_io_pmd')

        self.pub_state_joint_current = rospy.Publisher(f'/dvrk/{self.arm}/state_joint_current',
                                                       sensor_msgs.msg.JointState, queue_size=1)
        self.pub_state_gripper_current = rospy.Publisher(f'/dvrk/{self.arm}/state_gripper_current',
                                                         sensor_msgs.msg.JointState, queue_size=1)
        self.sub_state_joint_desired = rospy.Subscriber(f'/dvrk/{self.arm}{sim_suffix}/state_joint_desired',
                                                        sensor_msgs.msg.JointState, queue_size=1)

    def publish_states(self):
        pass


if __name__ == '__main__':
    r = rospy.Rate(100)
    io_pmd = IOPMD()
    while not rospy.is_shutdown():
        io_pmd.publish_states()
        r.sleep()
    rospy.spin()
