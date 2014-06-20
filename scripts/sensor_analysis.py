#!/usr/bin/env python
import rospy
#from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Imu
import numpy as np
from matplotlib import pyplot as plt
from scipy import fftpack # Use for freq. response 
#import cv2

class ImuMeasure:
    def __init__(self, n_readings=1000):
        self.nav_sub = rospy.Subscriber("/ardrone/imu", 
                                        Imu, self.callback_navdata)
        self.n_readings = n_readings
        self.start_seq = -1
        self.start_time = 0
        self.t = np.zeros(n_readings)
        self.orientation = np.zeros((n_readings, 4))
        self.linear_accel = np.zeros((n_readings, 3))
        self.angular_velocity = np.zeros((n_readings, 3))
        
    def record_imu_data(self, data, idx):
        self.t[idx] = data.header.stamp.secs - self.start_time
        self.orientation[idx,0] = data.orientation.x
        self.orientation[idx,1] = data.orientation.y
        self.orientation[idx,2] = data.orientation.z
        self.orientation[idx,3] = data.orientation.w
        self.linear_accel[idx,0] = data.linear_acceleration.x
        self.linear_accel[idx,1] = data.linear_acceleration.y
        self.linear_accel[idx,2] = data.linear_acceleration.z
        self.angular_velocity[idx,0] = data.angular_velocity.x
        self.angular_velocity[idx,1] = data.angular_velocity.y
        self.angular_velocity[idx,2] = data.angular_velocity.z
        
    def plot_imu_readings(self):
        plt.figure(); plt.grid()
        plt.plot(self.t, self.orientation)
        plt.legend(('x','y','z','w'))
        plt.title('Orientation vs. Time')
        plt.figure(); plt.grid()
        plt.plot(self.t, self.linear_accel)
        plt.legend(('x','y','z'))
        plt.title('Linear Accel vs. Time')
        plt.figure(); plt.grid()
        plt.plot(self.t, self.angular_velocity)
        plt.legend(('x','y','z'))
        plt.title('Angular Velocity vs. Time')
        plt.show()
            
    def callback_navdata(self, data):
        if self.start_seq == -1:
            self.start_seq = data.header.seq
            self.start_time = data.header.stamp.secs
        idx = data.header.seq - self.start_seq
        if  idx < self.n_readings:
            self.record_imu_data(data, idx)
        else:
            self.plot_imu_readings()
            
            
if __name__ == '__main__':
    rospy.init_node('ardrone_sensor_analysis')
    imu_measure = ImuMeasure(1000)
    rospy.spin()