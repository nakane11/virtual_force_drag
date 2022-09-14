#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty

class LeadFetch(object):

    def __init__(self):
        self.valid_duration = rospy.get_param('~valid_duration', 1)
        self.timer_running = False
        self.fx_threshold = 0.7
        self.v_max = 0.5
        self.status = 'STOP'
        self.fx = 0
        self.vx = 0

        self.pub = rospy.Publisher('~output', Twist, queue_size=1)
        self.sub = rospy.Subscriber('~input', WrenchStamped, self.wrench_cb)
        self.srv_start = rospy.Service('~start', Empty, self.start_timer)
        self.srv_stop = rospy.Service('~stop', Empty, self.stop_timer)
        
        self.rate = rospy.get_param('~rate', 70)
        if self.rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 70.')
            self.rate = 70
        self.last_updated_time = rospy.Time.now()

    def wrench_cb(self, msg):
        self.fx = (msg.wrench.force.x + 10)/60
        self.last_updated_time = rospy.Time.now()
        
    def timer_cb(self, timer):
        fx = self.fx
        elapsed_time = rospy.Time.now() - self.last_updated_time
        if elapsed_time.secs > self.valid_duration:
            return

        if self.status == 'STOP':
            rospy.loginfo('STOP')
            if fx > self.fx_threshold:
                self.status = 'ACCEL'
            return
                
        elif self.status == 'STEADY':
            rospy.loginfo('STEADY')
            if fx < -self.fx_threshold:
                self.status = 'DECEL'
            elif fx > self.fx_threshold:
                self.status = 'ACCEL'
            
        elif self.status == 'ACCEL':
            if fx < -self.fx_threshold:
                self.status = 'DECEL'
            elif not (fx > self.fx_threshold):
                self.status = 'STEADY'
            
        elif self.status == 'DECEL':
            elif fx > self.fx_threshold:
                self.status = 'ACCEL'
            elif not (fx < -self.fx_threshold):
                self.status = 'STEADY'

        if self.status == 'ACCEL':
            self.vx += self.acceleration_x(fx)
            rospy.loginfo('ACCEL: +{}'.format(self.acceleration_x(fx)))
            if self.vx > self.v_max:
                self.vx = self.v_max
        elif self.status == 'DECEL':
            self.vx += self.deceleration_x(fx)
            rospy.loginfo('DECEL: {}'.format(self.deceleration_x(fx)))
            if self.vx < 0:
                self.vx = 0
                self.status == 'STOP'
        self.send_cmd_vel(x=self.vx)
        
    def send_cmd_vel(self, x=0, d=0):
        pub_msg = Twist()
        pub_msg.linear.x = x
        pub_msg.angular.z = d
        self.pub.publish(pub_msg)

    def acceleration_x(self, f):
        return 0.005 * math.cos(1.5*f)

    def deceleration_x(self, f):
        return 0.1 * (1-math.exp((-f)/5))

    def start_timer(self, req):
        if not self.timer_running:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.timer_cb)
            self.timer_running = True
            rospy.loginfo('Timer started')
        else:
            rospy.loginfo('Already timer is running')
            
    def stop_timer(self, req):
        if self.timer_running:
            self.timer.shutdown()
            self.timer_running = False
            rospy.loginfo('Timer stopped')
        else:
            rospy.loginfo('Timer is not running yet')
            
if __name__ == '__main__':
    rospy.init_node('lead_fetch')
    LeadFetch()
    rospy.spin()
