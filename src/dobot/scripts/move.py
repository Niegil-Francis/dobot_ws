from __future__ import print_function	


import warnings
warnings.filterwarnings("ignore")
import cv2 as cv
import argparse
import numpy as np

import rospy
from geometry_msgs.msg import Pose
import time
from tkinter import *
from tkinter import filedialog
from PIL import ImageTk, Image
import cv2
import pytesseract 

import speech_recognition as sr 
import pyttsx3 


    
pub = rospy.Publisher('geometry_pose', Pose, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(1000) # 10hz

msg = Pose()				
msg.position.x = 227
msg.position.y = 9
msg.position.z = 101
pub.publish(msg)
time.sleep(1)
pub.publish(msg)
time.sleep(1)
					
