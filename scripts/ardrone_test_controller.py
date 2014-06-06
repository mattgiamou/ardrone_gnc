#!/usr/bin/env python
# Copyright (c) 2012, Falkor Systems, Inc. All rights reserved.
# Modified 2014 by Matthew Giamou

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer. Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution. THIS SOFTWARE IS
# PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class ArdroneTestController:
    def __init__( self ):
        self.nav_sub = rospy.Subscriber("ardrone/navdata", 
                                        Navdata, self.callback_navdata)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)
        
    def callback_navdata( self, data ):
        self.vx = data.vx/1e3
        self.vy = data.vy/1e3
        self.vz = data.vz/1e3

    def update( self ):
        if self.last_time == None:
            self.last_time = rospy.Time.now()
            dt = 0.0
        else:
            time = rospy.Time.now()
            dt = (time - self.last_time).to_sec()
            self.last_time = time
            
        # Keep still for now
        cmd = Twist()
        cmd.angular.z = 0.0
        cmd.angular.y = 0.0
        cmd.angular.x = 0.0
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        self.cmd_vel_pub.publish(cmd)

def main():
  rospy.init_node('ardrone_test_controller')

  control = ArdroneTestController()
  r = rospy.Rate(100)

  try:
      while not rospy.is_shutdown():
          control.update()
          r.sleep()
  except KeyboardInterrupt:
    print "Shutting down ardrone_test_controller node."

if __name__ == '__main__':
    main()