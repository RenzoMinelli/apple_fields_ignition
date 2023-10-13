#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
import time as Time


def process_data(data):
    rospy.loginfo(data)
    #f = open("data.txt", "x")
    #f.write(str(data))
   # f.close()
    rospy.signal_shutdown("test node has been shutdown")


if __name__ == '__main__':
    rospy.init_node('test_node')    
    sub = rospy.Subscriber("/zed_front/left/compressed", CompressedImage, process_data)
    #sub2 = rospy.Subscriber("/zed_lateral/zed_lateral/imu/data", Imu, process_data)
    #sub3 = rospy.Subscriber("/zed_front/zed_node_front/imu/data", Imu, process_data)

    rospy.loginfo('test node has been started...')
    rospy.spin() #blocks until node is shutdown, Yields activity on other threads