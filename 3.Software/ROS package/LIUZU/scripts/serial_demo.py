#!/usr/bin/env python
#coding=utf-8
import rospy
import serial
from geometry_msgs.msg import Twist
import time

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=2,rtscts=True,dsrdtr=True)
ser.isOpen()   
res=ser.readall() 

if __name__=="__main__":
  rospy.init_node('serial_demo')
  rate = rospy.Rate(1)
  count=0
  while 1:
    data=" liu"
    rospy.loginfo("This is my serial write demo program!")
    count+=1
    data=data+str(count)
    ser.write(data)
    rate.sleep()
