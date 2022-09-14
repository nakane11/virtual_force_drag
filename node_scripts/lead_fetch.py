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
        self.fx_threshold = 0.6
        self.fy_threshold = 0.6
        self.vx_max = 0.45
        self.vy_max = math.pi
        self.status_x = 'STOP'
        self.status_y = 'STOP'
        self.fx = 0
        self.fy = 0
        self.vx = 0
        self.vy = 0

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
        self.fx = (msg.wrench.force.x - 1.3)/5
        self.fy = (msg.wrench.force.y - 1.3)/5
        self.last_updated_time = rospy.Time.now()

    def execute_x(self, f):
        if self.status_x == 'STOP':
            rospy.loginfo('STOP')
            if f > self.fx_threshold:
                self.status_x = 'ACCEL'
            return
                
        elif self.status_x == 'STEADY':
            rospy.loginfo('STEADY')
            if f < -self.fx_threshold:
                self.status_x = 'DECEL'
            elif f > self.fx_threshold:
                self.status_x = 'ACCEL'
            
        elif self.status_x == 'ACCEL':
            if f < -self.fx_threshold:
                self.status_x = 'DECEL'
            elif not (f > self.fx_threshold):
                self.status_x = 'STEADY'
            
        elif self.status_x == 'DECEL':
            if f > self.fx_threshold:
                self.status_x = 'ACCEL'
            elif not (f < -self.fx_threshold):
                self.status_x = 'STEADY'

        if self.status_x == 'ACCEL':
            self.vx += self.acceleration_x(f)
            rospy.loginfo('ACCEL: +{}'.format(self.acceleration_x(f)))
            if self.vx > self.vx_max:
                self.vx = self.vx_max
        elif self.status_x == 'DECEL':
            self.vx += self.deceleration_x(f)
            rospy.loginfo('DECEL: {}'.format(self.deceleration_x(f)))
            if self.vx < 0:
                self.vx = 0
                self.status_x == 'STOP'

    def execute_y(self, f):
        if self.status_y == 'STOP':
            rospy.loginfo('STOP')
            if abs(f) > self.fy_threshold:
                self.status_y = 'ACCEL'
            return
                
        elif self.status_y == 'STEADY':
            rospy.loginfo('STEADY')
            if abs(f) > self.fy_threshold:
                if f * self.vy < 0:
                    self.status_y = 'DECEL'
                elif f * self.vy > 0:
                    self.status_y = 'ACCEL'
            
        elif self.status_y == 'ACCEL':
            if abs(f) > self.fy_threshold:
                if f * self.vy < 0:
                    self.status_y = 'DECEL'
            elif abs(f) < self.fy_threshold:
                self.status_y = 'STEADY'
            
        elif self.status_y == 'DECEL':
            if abs(f) > self.fy_threshold:
                if f * self.vy < 0:
                self.status_y = 'DECEL'
            elif abs(f) < self.fy_threshold:
                self.status_y = 'STEADY'

        if self.status_y == 'ACCEL':
            self.vy += self.acceleration_y(f)
            rospy.loginfo('ACCEL: +{}'.format(self.acceleration_y(f)))
            if self.vy > self.vy_max:
                self.vy = self.vy_max
        elif self.status_y == 'DECEL':
            self.vy += self.deceleration_y(f)
            rospy.loginfo('DECEL: {}'.format(self.deceleration_y(f)))
            if self.vy < 0:
                self.vy = 0
                self.status_y == 'STOP'
                
    def timer_cb(self, timer):
        fx = self.fx
        fy = self.fy
        elapsed_time = rospy.Time.now() - self.last_updated_time
        if elapsed_time.secs > self.valid_duration:
            return

        # self.execute_x(fx)
        self.execute_y(fy)
        self.send_cmd_vel(x=self.vx, d=self.vy)
        
    def send_cmd_vel(self, x=0, d=0):
        pub_msg = Twist()
        pub_msg.linear.x = x
        pub_msg.angular.z = d
        self.pub.publish(pub_msg)

    def acceleration_x(self, f):
        return 0.003 * math.log(1.5*f)

    def deceleration_x(self, f):
        return 0.1 * (1-math.exp((-f)/5))

    def acceleration_y(self, f):
        return 0.003 * math.log(1.5*f)

    def deceleration_y(self, f):
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
