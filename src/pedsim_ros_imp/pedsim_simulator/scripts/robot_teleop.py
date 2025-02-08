#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist


MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 1.2

LIN_VEL_STEP_SIZE = 0.04
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity ( ~ 2.0)
a/d : increase/decrease angular velocity ( ~ 2.0)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

class Robot:
    def __init__(self):
        # rospy.init_node('pedsim_keyboard_teleop')
        self.status = 0
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0


    def loopHandleKeys(self, keys):
        # try:
        if keys[0] == 'w' :
            self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            print(self.vels(self.target_linear_vel,self.target_angular_vel))
        elif keys[0] == 'x' :
            self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            print(self.vels(self.target_linear_vel,self.target_angular_vel))
        if keys[1] == 'a' :
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
            self.status = self.status + 1
            print(self.vels(self.target_linear_vel,self.target_angular_vel))
        elif keys[1] == 'd' :
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
            self.status = self.status + 1
            print(self.vels(self.target_linear_vel,self.target_angular_vel))
        if keys[0]=='s':
            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = 0.0
            self.control_angular_vel = 0.0
            print(self.vels(self.target_linear_vel, self.target_angular_vel))

        if self.status == 20 :
            print(msg)
            self.status = 0

        twist = Twist()

        self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = self.control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.control_angular_vel
        return twist
        # self.pub.publish(twist)

        # except:
        #     print(e)

        # finally:
        #     twist = Twist()
        #     twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        #     twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        #     return twist

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

        return output

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input
        return input

    def checkLinearLimitVelocity(self, vel):
        vel = self.constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        return vel

    def checkAngularLimitVelocity(self, vel):
        vel = self.constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
        return vel