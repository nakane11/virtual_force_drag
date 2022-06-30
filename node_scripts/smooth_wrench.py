#!/usr/bin/env python

from geometry_msgs.msg import WrenchStamped
import numpy as np
import rospy

from jsk_topic_tools import ConnectionBasedTransport


class SmoothWrench(ConnectionBasedTransport):

    def __init__(self):
        super(SmoothWrench, self).__init__()
        self._length = rospy.get_param('~length', 10)
        self.wrench_array = np.zeros((self._length, 6))
        self.pub = self.advertise('~output', WrenchStamped, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input',
            WrenchStamped, queue_size=1,
            callback=self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def update(self, array):
        self.wrench_array = np.append(self.wrench_array, array, axis=0)
        self.wrench_array = np.delete(self.wrench_array, 0, axis=0)
        return np.average(self.wrench_array, axis=0)
        
    def _cb(self, msg):
        input_array = np.array([[msg.wrench.force.x,
                                msg.wrench.force.y,
                                msg.wrench.force.z,
                                msg.wrench.torque.x,
                                msg.wrench.torque.y,
                                msg.wrench.torque.z]])
        ave = self.update(input_array)
        pub_msg = WrenchStamped(header=msg.header)
        pub_msg.wrench.force.x = ave[0]
        pub_msg.wrench.force.y = ave[1]
        pub_msg.wrench.force.z = ave[2]
        pub_msg.wrench.torque.x = ave[3]
        pub_msg.wrench.torque.y = ave[4]
        pub_msg.wrench.torque.z = ave[5]
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('smooth_wrench')
    smooth_wrench = SmoothWrench()  # NOQA
    rospy.spin()
