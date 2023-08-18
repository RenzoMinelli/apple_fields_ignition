#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import subprocess


class Nodo(object):
    # Nodo de ROS que captura imágenes del tópico "/costar_husky_sensor_config_1/left/image_raw" 
    # y las guarda en una carpeta de frames, también graba un video de las imágenes capturadas.
    counter = 0
   
    def __init__(self, field, experiment):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        self.field = field
        self.experiment = experiment
        fields_ignition_path = subprocess.run(['rospack', 'find', 'fields_ignition'], stdout=subprocess.PIPE).stdout.decode("utf-8").replace('\n', '')
        self.path = os.path.expanduser("{}/generated/{}/experiments/{}/frames".format(fields_ignition_path,self.field, self.experiment))
        os.makedirs(self.path, exist_ok=True)
        self.result = cv2.VideoWriter('{}/generated/{}/experiments/{}/record.mp4'.format(fields_ignition_path, self.field, self.experiment), 
                         cv2.VideoWriter_fourcc(*'mp4v'),
                         10, (1920, 1080))
        
        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/costar_husky_sensor_config_1/left/image_raw",Image,self.callback)


    def callback(self, msg):
        image = self.br.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("{}/frame_{}.jpg".format(self.path, self.counter), image)
        self.result.write(image)
        self.counter += 1
        
    def start(self):
        #rospy.spin()
        while not rospy.is_shutdown():
            pass
        print("FINISH")
        self.result.release()
        cv2.destroyAllWindows()
        print("The video was successfully saved")

if __name__ == '__main__':
    rospy.init_node('field_recorder', anonymous=True)
    field = rospy.get_param('~field_name', True)
    experiment = rospy.get_param('~experiment_name')
    node = Nodo(field, experiment)
    node.start()