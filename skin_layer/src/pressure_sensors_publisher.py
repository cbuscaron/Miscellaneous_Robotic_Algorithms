#!/usr/bin/env python
'''
Camilo F. Buscaron
2/1/2017
ROS Node to publish pressure sensor array data coming from MCU connected via USB
'''

from __future__ import print_function
import serial

import rospy
from std_msgs.msg import Float64

import os, sys

'''
3b_pad_z+_link
3b_pad_z+_joint
3t_pad_z+_link
3t_pad_z+_joint
3b_pad_z-_link
3b_pad_z-_joint
3t_pad_z-_link
3t_pad_z-_joint
3t_pad_y+_link
3t_pad_y+_joint
3b_pad_y+_link
3b_pad_y+_joint
3t_pad_y-_link
3t_pad_y-_joint
3b_pad_y-_link
3b_pad_y-_joint

4b_pad_z+_link
4b_pad_z+_joint
4t_pad_z+_link
4t_pad_z+_joint
4b_pad_z-_link
4b_pad_z-_joint
4t_pad_z-_link
4t_pad_z-_joint
4b_pad_y+_link
4b_pad_y+_joint
4t_pad_y+_link
4t_pad_y+_joint
4b_pad_y-_link
4b_pad_y-_joint
4t_pad_y-_link
4t_pad_y-_joint
'''

def main():
    pad_1 = rospy.Publisher('3b_pad_z+', Float64, queue_size=100)
    pad_2 = rospy.Publisher('3t_pad_z+', Float64, queue_size=100)
    pad_3 = rospy.Publisher('3b_pad_z-', Float64, queue_size=100)
    pad_4 = rospy.Publisher('3t_pad_z-', Float64, queue_size=100)

    pad_5 = rospy.Publisher('3b_pad_y+', Float64, queue_size=100)
    pad_6 = rospy.Publisher('3t_pad_y+', Float64, queue_size=100)
    pad_7 = rospy.Publisher('3b_pad_y-', Float64, queue_size=100)
    pad_8 = rospy.Publisher('3t_pad_y-', Float64, queue_size=100)

    pads_list = [pad_1, pad_2, pad_3, pad_4, pad_5, pad_6, pad_7, pad_8]

    rospy.init_node('pressure_sensors_publisher_node')
    rate = rospy.Rate(1000) # 1Khz

    ser = serial.Serial('/dev/ttyACM0', 115200)

    try:
        while not rospy.is_shutdown():
            try:
                readings = ser.readline().strip('\r\n')
                sensors = readings.split(' ')
                del sensors[-1]

                for i, sensor in enumerate(sensors):
                    pads_list[i].publish(float(sensors[i]))

                rate.sleep()

            except ValueError, e:
                continue

    except KeyboardInterrupt:
        ser.close()
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

if __name__ == '__main__':
    main()
