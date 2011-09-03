#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, UC Regents
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of California nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('jh_fly')
import rospy
import numpy as np
from math import sin, cos, radians, degrees
import tf.transformations as tft

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from flyer_controller.msg import control_mode_output
from ardrone_brown.msg import Navdata

from starmac_roslib.timers import Timer
from starmac_roslib.pid import PidController


class ArAdapter(object):

    def __init__(self):
        pass
        
    def start(self):
        rospy.init_node('ar_adapter')
        #self.init_params()
        #self.init_state()
        self.init_vars()
        self.init_publishers()
        self.init_subscribers()
        rospy.spin()
        
    def init_vars(self):
        self.latest_cmd_msg = control_mode_output()
        self.motor_enable = False
        self.thrust_cmd = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.lastFrameTime = rospy.Time.now()
        
    def init_state(self):
        if self.model == "simple_nonlinear":
            self.model = SimpleNonlinearModel()
        elif self.model == "linear":
            self.model = LinearModel()
        else:
            rospy.logerror("Model type '%s' unknown" % self.model)
            raise Exception("Model type '%s' unknown" % self.model)
        
    def init_params(self):
        self.model = rospy.get_param("~model", "simple_nonlinear")

    def init_publishers(self):
        # Publishers
        self.pub_odom = rospy.Publisher('odom', Odometry)
        
    def init_subscribers(self):
        # Subscribers
        self.control_input_sub = rospy.Subscriber('/ardrone/navdata', Navdata, self.update_state)
        #self.motor_enable_sub = rospy.Subscriber('teleop_flyer/motor_enable', Bool, self.motor_enable_callback)
      
    
    def motor_enable_callback(self, msg):
        if msg.data != self.motor_enable:
            #rospy.loginfo('Motor enable: ' + str(msg.data))
            self.motor_enable = msg.data
    
    def update_state(self, msg):
	##print 'Current command is: ' + str(msg)
	tnow = rospy.Time.now()
	dt = tnow.to_sec() - self.lastFrameTime.to_sec()
	self.lastFrameTime = tnow
	## print 'dt = ' + str(dt)
	self.x = self.x + (msg.vx * dt)
	self.y = self.y + (msg.vy * dt)
	self.z = msg.altd
	print 'x y z '+ str(self.x) +' '+ str(self.y) +' '+ str(self.z)
        self.publish_odometry()
            
    def update_controller(self, dt):
        #lcm = self.latest_cmd_msg
        #self.model.set_input(np.array([lcm.yaw_cmd, lcm.pitch_cmd, lcm.roll_cmd, lcm.alt_cmd, lcm.motors_on]), dt)
        rospy.loginfo("thrust_cmd = %f, dt = %f" % (self.thrust_cmd, dt))
                    
    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "/ned"
        oppp = odom_msg.pose.pose.position
        oppp.x, oppp.y, oppp.z  = self.x, self.y, -self.z #self.model.get_position()
        ottl = odom_msg.twist.twist.linear
        ottl.x, ottl.y, ottl.z = 0,0,0 #self.model.get_velocity()
        oppo = odom_msg.pose.pose.orientation
        oppo.x, oppo.y, oppo.z, oppo.w = 1,1,1,1 #self.model.get_orientation()
        otta = odom_msg.twist.twist.angular
        otta.x, otta.y, otta.z = 0.1,0.1,0.1 #self.model.get_angular_velocity()
        self.pub_odom.publish(odom_msg)
        
      
if __name__ == "__main__":
  self = ArAdapter()
  self.start()