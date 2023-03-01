#! /usr/bin/env python3

"""
Copyright (c) 2023 Mohammad Nazeri, Chenhui Pan
"""
import sys
sys.path.append('../nn/')
import math
from math import sin, cos, pi
from collections import deque
import rospy
import numpy as np
import tf
import cv2
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import torch
import torch.nn as nn
#import torchvision
#from torchvision import transforms
#from PIL import Image
#from utils import BEVLidar, get_affine_mat, get_affine_matrix_quat
from net import NeuralNetwork
from torchvision.models import resnet18, ResNet18_Weights
from torchvision import transforms
import matplotlib.pyplot as plt


class ActionGenerator:

    def __init__(self):
        self.cvbridge = CvBridge()
        self.device = "cuda:0"
        #weights = ResNet18_Weights.DEFAULT
        #encoder = resnet18(weights=weights)
        #encoder.fc = nn.Identity()
        #encoder.conv1 = nn.Conv2d(1, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False)
        self.model = torch.load('/home/robotixx/crawlingcar_ws/src/Xman_Autonomous_Crawler/navstack_pub/src/model_crawler.pt').to(self.device)
        self.model.eval()
        self.cam = torch.zeros((224 , 224), dtype=torch.float)
        self.preprocess = transforms.Compose([transforms.ToTensor(), transforms.Resize(224)])


    def camCB(self, data):

        #cv2image = self.cvbridge.compressed_imgmsg_to_cv2(data, 'passthrough')
        #cv2image = np.frombuffer(data.data, np.uint8)
        cv2image = np.frombuffer(data.data, np.uint8)
        
        cv2image = cv2.imdecode(cv2image, cv2.IMREAD_GRAYSCALE).astype(float)
        
        cv2image = cv2.normalize(cv2image, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        #plt.imshow(cv2image, cmap='gray')
        #plt.show()
        #print(f"{type(cv2image) = }")
        #print(f"{cv2image.shape = }")
        #print(f"{cv2image.min() = }")
        #print(f"{cv2image.max() = }")
        cv2image = self.preprocess(cv2image).float()
        


        
        
        #normed_img = transforms.ToTensor()(cv2image)
        #normed_img = (normed_img - normed_img.min()) / (normed_img.max() - normed_img.min())
        #normed_img = transforms.Resize(224)(normed_img).squeeze()
        #cv2.imwrite(f"image/{data.header.stamp.to_sec()}.png", cv2image.clone().unsqueeze(0).numpy())
        #print("In camera callback")
        #print(f"{cv2image.shape = }")
        #print(f"{cv2image.min() = }")
        #print(f"{cv2image.max() = }")
        #self.cam = torch.from_numpy(normed_img).float()
        self.cam = cv2image.squeeze()
        #print(f"{self.cam.shape = }")

    @torch.no_grad()
    def compute_command(self):
        #print(f"{self.cam = }")
        action = self.model(self.cam.unsqueeze(0).unsqueeze(0).to(self.device))
        print(f"{action = }")

        return action.cpu().squeeze()

### Main ###

if __name__ == '__main__':
    rospy.init_node('action_generator', anonymous=True)
    generator = ActionGenerator()
    action = Twist()
    action_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) # queue_size = 50
    cam_sub = rospy.Subscriber("depth/image_raw/compressed", CompressedImage, generator.camCB)
    while not rospy.is_shutdown():
        vw = generator.compute_command()
        action.linear.x = vw[1]
        action.angular.z = vw[0]
        action_pub.publish(action)
