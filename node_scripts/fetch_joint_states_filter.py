#!/usr/bin/env python
from sensor_msgs.msg import JointState
import rospy
from jsk_topic_tools import ConnectionBasedTransport

class JointStatesFilter(ConnectionBasedTransport):

    def __init__(self):
        super(JointStatesFilter, self).__init__()
        self.pub = self.advertise('/joint_states_filtered', JointState, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('/joint_states', JointState, self.cb)

    def unsubscribe(self):
        self.sub.unregister()

    def cb(self, msg):
        if len(msg.name) > 2:
            self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('joint_states_filter')
    JointStatesFilter()
    rospy.spin()

