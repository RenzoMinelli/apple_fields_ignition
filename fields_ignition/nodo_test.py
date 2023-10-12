#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node('test_node')    
    rospy.loginfo('test node has been started...')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown(): #si apretamos ctrl c esto se vuelve false 
        rospy.loginfo('hello')
        rate.sleep()
