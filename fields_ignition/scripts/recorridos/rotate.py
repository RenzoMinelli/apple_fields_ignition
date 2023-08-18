#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math

roll = pitch = yaw = 0.0
target_angle = 120
kP = 0.5

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/costar_husky_sensor_config_1/odom', Odometry, get_rotation)
pub = rospy.Publisher('/costar_husky_sensor_config_1/cmd_vel', Twist, queue_size=1)
command= Twist()
r = rospy.Rate(50)
while not rospy.is_shutdown():
    target_rad = target_angle * math.pi/180
    if(target_rad - yaw > 0.01): # 1 grado
        command.angular.z = kP  * (target_rad - yaw)
        print(target_rad - yaw)
        pub.publish(command)
        r.sleep()
    else: 
        break