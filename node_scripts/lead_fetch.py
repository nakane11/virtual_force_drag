#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool

class LeadFetch(object):

    def __init__(self):
        self.valid_duration = rospy.get_param('~valid_duration', 1)
        self.timer_running = False
        self.fx_threshold = 0.6
        self.fy_threshold = 0.7
        self.vx_max = 0.7
        self.vy_max = math.pi * 0.3
        self.reset()
        self.pub = rospy.Publisher('~output', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('~status', Bool, queue_size=1, latch=True)
        self.sub = rospy.Subscriber('~input', WrenchStamped, self.wrench_cb)
        self.srv_start = rospy.Service('~start', Empty, self.start_timer)
        self.srv_stop = rospy.Service('~stop', Empty, self.stop_timer)
        
        self.rate = rospy.get_param('~rate', 70)
        if self.rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 70.')
            self.rate = 70
        self.last_updated_time = rospy.Time.now()

    def reset(self):
        self.status_x = 'STOP'
        self.status_y = 'STOP'
        self.fx = 0
        self.fy = 0
        self.vx = 0
        self.vy = 0
        self.sign_x = 1
        self.sign_y = 1

    def wrench_cb(self, msg):
        self.fx = (msg.wrench.force.x - 1.5)/7
        # self.fy = (msg.wrench.force.y - 0.8)/3.5
        self.last_updated_time = rospy.Time.now()

    def execute_x(self, f):
        if self.status_x == 'STOP':
            rospy.loginfo('STOP')
            if abs(f) > self.fx_threshold:
                self.status_x = 'ACCEL'
                self.sign_x = 1 if f > 0 else -1
            return
                
        elif self.status_x == 'STEADY':
            rospy.loginfo('STEADY')
            if self.vx > 0:
                self.sign_x = 1
            elif self.vx < 0:
                self.sign_x = -1
            else:
                self.sign_x = 1 if f > 0 else -1
            if abs(f) > self.fx_threshold:
                if f * self.sign_x >= 0:
                    self.status_x = 'ACCEL'
                else:
                    self.status_x = 'DECEL'
            
        elif self.status_x == 'ACCEL':
            if abs(f) < self.fx_threshold:
                self.status_x = 'STEADY'
            
        elif self.status_x == 'DECEL':
            if abs(f) < self.fx_threshold:
                self.status_x = 'STEADY'

        if self.status_x == 'ACCEL':
            self.vx += self.sign_x * self.acceleration_x(f)
            rospy.loginfo('ACCEL: {}'.format(self.acceleration_x(f)))
        elif self.status_x == 'DECEL':
            self.vx -= self.sign_x * self.deceleration_x(f)
            rospy.loginfo('DECEL: {}'.format(self.deceleration_x(f)))
            if abs(self.vx) < 0.03:
                self.vx = 0
                self.status_x == 'STOP'
        self.sign_x = 1 if self.vx > 0 else -1
        if abs(self.vx) > self.vx_max:
            self.vx = self.sign_x * self.vx_max

    # def execute_y(self, f):
    #     if self.status_y == 'STOP':
    #         rospy.loginfo('STOP')
    #         if abs(f) > self.fy_threshold:
    #             self.status_y = 'ACCEL'
    #         return
                
    #     elif self.status_y == 'STEADY':
    #         rospy.loginfo('STEADY')
    #         if abs(f) > self.fy_threshold:
    #             if f * self.vy < 0:
    #                 self.status_y = 'DECEL'
    #             elif f * self.vy > 0:
    #                 self.status_y = 'ACCEL'
            
    #     elif self.status_y == 'ACCEL':
    #         if abs(f) > self.fy_threshold:
    #             if f * self.vy < 0:
    #                 self.status_y = 'DECEL'
    #         elif abs(f) < self.fy_threshold:
    #             self.status_y = 'STEADY'
            
    #     elif self.status_y == 'DECEL':
    #         if abs(f) > self.fy_threshold:
    #             if f * self.vy < 0:
    #                 self.status_y = 'DECEL'
    #         elif abs(f) < self.fy_threshold:
    #             self.status_y = 'STEADY'

    #     if self.status_y == 'ACCEL':
    #         self.vy += self.acceleration_y(f)
    #         rospy.loginfo('ACCEL: +{}'.format(self.acceleration_y(f)))
    #         if abs(self.vy) > self.vy_max:
    #             self.vy = (1 if f>0 else -1)*self.vy_max
    #     elif self.status_y == 'DECEL':
    #         self.vy += self.deceleration_y(f)
    #         rospy.loginfo('DECEL: {}'.format(self.deceleration_y(f)))
                
    def timer_cb(self, timer):
        fx = self.fx
        fy = self.fy
        elapsed_time = rospy.Time.now() - self.last_updated_time
        if elapsed_time.secs > self.valid_duration:
            return

        self.execute_x(fx)
        # self.execute_y(fy)
        self.send_cmd_vel(x=self.vx, d=self.vy)
        
    def send_cmd_vel(self, x=0, d=0):
        pub_msg = Twist()
        pub_msg.linear.x = x
        pub_msg.angular.z = d
        self.pub.publish(pub_msg)

    def acceleration_x(self, f):
        # return 0.004 * math.log(1.5*abs(f))
        return 0.004 * math.atan(abs(f))

    def deceleration_x(self, f):
        return 0.002 * math.exp(3*abs(f))

    # def acceleration_y(self, f):
    #     return (1 if f>0 else -1)*0.003 * math.log(1.5*abs(f))

    # def deceleration_y(self, f):
    #     return (1 if f>0 else -1)*0.1 * (1-math.exp((-abs(f))/5))

    def start_timer(self, req):
        if not self.timer_running:
            self.reset()
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.timer_cb)
            self.timer_running = True
            rospy.loginfo('Timer started')
        else:
            rospy.loginfo('Already timer is running')
        self.status_pub.publish(Bool(data=True))
        return EmptyResponse()
            
    def stop_timer(self, req):
        if self.timer_running:
            self.timer.shutdown()
            self.timer_running = False
            rospy.loginfo('Timer stopped')
        else:
            rospy.loginfo('Timer is not running yet')
        self.status_pub.publish(Bool(data=False))
        return EmptyResponse()
            
if __name__ == '__main__':
    rospy.init_node('lead_fetch')
    LeadFetch()
    rospy.spin()
