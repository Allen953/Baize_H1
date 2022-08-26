#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
import sys


if __name__=="__main__":
  rospy.init_node('teleop_demo')
  rate = rospy.Rate(1)
  while 1:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
		#不产生回显效果
    old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
    try :
        tty.setraw( fd )
        ch = sys.stdin.read( 1 )
    finally :
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    rospy.loginfo("This is my serial write demo program!"+str(ch))

    
    
    rate.sleep()
