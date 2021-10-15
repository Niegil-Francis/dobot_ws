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






def extract(image):
    Actual_image = image
    #Sample_img = cv2.resize(Actual_image,(400,350))
    #Image_ht,Image_wd,Image_thickness = Sample_img.shape
    #Sample_img = cv2.cvtColor(Sample_img,cv2.COLOR_BGR2RGB)
    texts = pytesseract.image_to_string(image) 
    return(texts)

# Python program to translate 
# speech to text and text to speech 


# Initialize the recognizer 
r = sr.Recognizer() 

# Function to convert text to 
# speech 
def SpeakText(command): 
	# Initialize the engine 
	engine = pyttsx3.init() 
	engine.say(command) 
	engine.runAndWait() 
	
def speech_recog():
	r = sr.Recognizer()
	with sr.Microphone() as source:

	  r.adjust_for_ambient_noise(source)
	  print("Say Yes or No")
	  audio = r.listen(source)

	# recognize speech using Google Speech Recognition
	try:
		# for testing purposes, we're just using the default API key
		# to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
		# instead of `r.recognize_google(audio)`
		out=r.recognize_google(audio)
		print("Dobot thinks you said " + out)
		if(out=='yes'):
			return 1
		elif(out=='no'):
			return 2
		else:
			return 3
	except:
		pass
	



max_value = 255
max_value_H = 360//2
low_H = 67
low_S = 71
low_V = 30
high_H = 111
high_S = max_value
high_V = max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
window_contour_name = 'Contour'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)

def coordinates(match):
	up=-15
	dn=-60
	d={}
	d["A"]=np.array([[-1,0,up],[-1,0,dn],[-2.5,-3,dn],[-4,0,dn]])

	d["B"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[-3,-1.5,dn],[-2,-1.5,dn],[-3,-1.5,dn],[-3,0,dn],[0,0,dn],[0,0,up],[-1,-3,up],[-1,-3,dn],[-1,0,dn]])

	d["C"]=np.array([[-4,-3,up],[-4,-3,dn],[-1,-3,dn],[-1,0,dn],[-4,0,dn]])

	d["D"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[-3,0,dn],[0,0,dn],[0,0,up],[-1,-3,up],[-1,-3,dn],[-1,0,dn]])

	d["E"]=np.array([[-3,-3,up],[-3,-3,dn],[-1,-3,dn],[-1,0,dn],[-1,0,dn],[-3,0,dn],[-3,0,up],[-3,-1.5,up],[-3,-1.5,dn],[-1,-1.5,dn]])

	d["F"]=np.array([[-3,-3,up],[-3,-3,dn],[-1,-3,dn],[-1,0,dn],[-1,0,up],[-3,-1.5,up],[-3,-1.5,dn],[-1,-1.5,dn]])


	d["G"]=np.array([[-3,-3,up],[-3,-3,dn],[-1,-3,dn],[-1,0,dn],[-3,0,dn],[-3,-1.5,dn]])

	d["H"]=np.array([[-4,-3,up],[-4,-3,dn],[-4,0,dn],[-4,0,up],[-4,-1.5,up],[-4,-1.5,dn],[-1,-1.5,dn],[-1,-1.5,up],[-1,-3,up],[-1,-3,dn],[-1,0,dn]])

	d["I"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[-3,-3,up],[-1.5,-3,up],[-1.5,-3,dn],[-1.5,0,dn],[-1.5,0,up],[-3,0,up],[-3,0,dn],[0,0,dn]])


	d["J"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[-3,-3,up],[-1.5,-3,up],[-1.5,-3,dn],[-1.5,0,dn],[0,0,dn]])

	d["K"]=np.array([[-1,-3,up],[-1,-3,dn],[-1,0,dn],[-1,0,up],[-4,-3,up],[-4,-3,dn],[-1,-1.5,dn],[-4,0,dn]])

	d["L"]=np.array([[-1,-3,up],[-1,-3,dn],[-1,0,dn],[-4,0,dn]])

	d["M"]=np.array([[-1,0,up],[-1,0,dn],[-1,-3,dn],[-2.5,-1.5,dn],[-4,-3,dn],[-4,0,dn]])

	d["N"]=np.array([[-1,0,up],[-1,0,dn],[-1,-3,dn],[-4,0,dn],[-4,-3,dn]])

	d["O"]=np.array([[-1,0,up],[-1,0,dn],[-1,-3,dn],[-4,-3,dn],[-4,0,dn],[-1,0,dn]])

	d["P"]=np.array([[-1,0,up],[-1,0,dn],[-1,-3,dn],[-3,-3,dn],[-3,-1.5,dn],[-1,-1.5,dn]])

	d["Q"]=np.array([[-4,0,up],[-4,0,dn],[-1,0,dn],[-1,-3,dn],[-4,-3,dn],[-4,0,dn],[-2.5,-1.5,dn]])

	d["R"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[-3,-1.5,dn],[-1,-1.5,dn],[-3,0,dn],[-3,0,up],[-1,0,up],[-1,0,dn],[-1,-3,dn]])

	d["S"]=np.array([[-4,-3,up],[-4,-3,dn],[-1,-3,dn],[-1,-1.5,dn],[-4,-1.5,dn],[-4,0,dn],[-1,0,dn]])

	d["T"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[-3,-3,up],[-1.5,-3,up],[-1.5,-3,dn],[-1.5,0,dn]])

	d["U"]=np.array([[-1,-3,up],[-1,-3,dn],[-1,0,dn],[-3,0,dn],[-3,-3,dn]])

	d["V"]=np.array([[0,-3,up],[0,-3,dn],[-1.5,0,dn],[-3,-3,dn]])

	d["W"]=np.array([[-1,-3,up],[-1,-3,dn],[-1,0,dn],[-2.5,-1.5,dn],[-4,0,dn],[-4,-3,dn]])

	d["X"]=np.array([[-3,-3,up],[-3,-3,dn],[0,0,dn],[0,0,up],[0,-3,up],[0,-3,dn],[-3,0,dn]])

	d["Y"]=np.array([[0,-3,up],[0,-3,dn],[-1.5,-1.5,dn],[-1.5,-1.5,up],[-3,-3,up],[-3,-3,dn],[0,0,dn]])

	d["Z"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[0,0,dn],[-3,0,dn]])


	d["1"]=np.array([[-1.5,-3,up],[-1.5,-3,dn],[-1.5,0,dn]])

	d["2"]=np.array([[-1,-3,up],[-1,-3,dn],[-4,-3,dn],[-4,-1.5,dn],[-1,-1.5,dn],[-1,0,dn],[-4,0,dn]])

	d["3"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[-3,0,dn],[0,0,dn],[0,0,up],[0,-1.5,up],[0,-1.5,dn],[-3,-1.5,dn]])

	d["4"]=np.array([[-1,-3,up],[-1,-3,dn],[-1,-1.5,dn],[-4,-1.5,dn],[-4,-1.5,up],[-4,-3,up],[-4,-3,dn],[-4,0,dn]])

	d["5"]=np.array([[-4,-3,up],[-4,-3,dn],[-1,-3,dn],[-1,-1.5,dn],[-4,-1.5,dn],[-4,0,dn],[-1,0,dn]])

	d["6"]=np.array([[-4,-3,up],[-4,-3,dn],[-1,-3,dn],[-1,0,dn],[-4,0,dn],[-4,-1.5,dn],[-1,-1.5,dn]])

	d["7"]=np.array([[0,-3,up],[0,-3,dn],[-3,-3,dn],[-3,0,dn]])

	d["8"]=np.array([[-1,-3,up],[-1,-3,dn],[-4,-3,dn],[-4,0,dn],[-1,0,dn],[-1,-3,dn],[-1,-3,up],[-1,-1.5,up],[-1,-1.5,dn],[-4,-1.5,dn]])

	d["9"]=np.array([[-4,0,up],[-4,0,dn],[-4,-3,dn],[-1,-3,dn],[-1,-1.5,dn],[-4,-1.5,dn]])

	d["0"]=np.array([[-1,-3,up],[-1,-3,dn],[-4,-3,dn],[-4,0,dn],[-1,0,dn],[-1,-3,dn]])

	d["/"]=np.array([[-1,0,up],[-1,0,dn],[-3,-3,dn],[-3,-3,up],[-4,-3,up]])

	d[" "]=np.array([[-2,0,up]])

	return d[match]

parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
cap = cv.VideoCapture(args.camera)
cv.namedWindow(window_capture_name)
#cv.namedWindow(window_detection_name)
'''
cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
'''
print("Press q/esc to quit and y to start writing")
while True:
    
	ret, frame = cap.read()
	if frame is None:
		break
	frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
	frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))


	cv.imshow(window_capture_name, frame)
	#cv.imshow(window_detection_name, frame_threshold)

	
	#im2, cntrs, hierarchy = cv.findContours(frame_threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
	#cntrs=np.array(cntrs)

	#contoured=cv.drawContours(frame, cntrs, -1, (0,255,0), 1)
	#cv.imshow(window_contour_name, contoured)


	ret, frame = cap.read()
	if frame is None:
		break	


	#gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 

	# Performing OTSU threshold 
	#ret, thresh1 = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV) 
	  
	# Specify structure shape and kernel size.  
	# Kernel size increases or decreases the area  
	# of the rectangle to be detected. 
	# A smaller value like (10, 10) will detect  
	# each word instead of a sentence. 
	rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (18, 18)) 
	  
	# Appplying dilation on the threshold image 
	#dilation = cv2.dilate(thresh1, rect_kernel, iterations = 1) 
	#cv2.imshow("Cropped Img",thresh1)
	txt = pytesseract.image_to_string(frame) 
	print(txt)
	'''
	# Finding contours 
	_,contours,_  = cv2.findContours(thresh1, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) 
	  
	# Creating a copy of image 
	im2 = frame.copy() 


	# Looping through the identified contours 
	# Then rectangular part is cropped and passed on 
	# to pytesseract for extracting text from it 
	for cnt in contours: 
	    x, y, w, h = cv2.boundingRect(cnt) 
	      
	    # Drawing a rectangle on copied image 
	    rect = cv2.rectangle(im2, (x, y), (x + w, y + h), (0, 255, 0), 2) 
	      
	    # Cropping the text block for giving input to OCR 
	    cropped = im2[y:y + h, x:x + w] 

	    
	      
	    # Apply OCR on the cropped image 
	'''
	

	key = cv.waitKey(30)
	if key == ord('q') or key == 27:
		break
	elif key==ord('y'):
		try: 
			
			strings=list(filter(None, txt.splitlines()))

			SpeakText("Did you write")
			SpeakText(strings[0])
			SpeakText(strings[1])
			SpeakText(strings[2])
			SpeakText("Yes or No?")
			output=speech_recog()
			
			if(output==3):
				print("Type y or n, dobot was not able to understand you correctly")
				output=input()
				
				
			if(output==1 or output=='y'):
				pub = rospy.Publisher('geometry_pose', Pose, queue_size=10)
				rospy.init_node('talker', anonymous=True)
				rate = rospy.Rate(1000) # 10hz

				msg = Pose()
			
				"""
				#Starting point calibration
				msg = Pose()

				msg.position.x = 30 #starting point 170 (140 max)
				msg.position.y = 100 #starting point -50 (140 max)
				msg.position.z = -50

				pub.publish(msg)
				time.sleep(0.3)
				
				total_x=200
				total_y=200

				scale=0.8
				total_x= (MAX[1] - MIN[1])*scale + 200
				total_y= (MAX[0] - MIN[0])*scale - 60

				while(total_x>300 or total_y>80):
					scale-=0.01
					total_x= (MAX[1] - MIN[1])*scale + 180
					total_y= (MAX[0] - MIN[0])*scale - 60
					
					
					
				
				for cnt in cntrs:
					j=0
					msg = Pose()
					if(len(cnt)<=5):
						continue
					print(len(cnt))
					for cnt1 in cnt:
						
						print("x:",(cnt1[0][1] - MIN[1])*0.5 + 170)
						print("y:", (cnt1[0][0] - MIN[0])*0.5 - MIN[0] - 60)
						if(j==0):
							msg.position.x = (cnt1[0][1] - MIN[1])*scale + 180 
							msg.position.y = (cnt1[0][0] - MIN[0])*scale - 60
							msg.position.z = 15
							pub.publish(msg)
							time.sleep(0.2)

						
						msg.position.x = (cnt1[0][1] - MIN[1])*scale + 180
						msg.position.y = (cnt1[0][0] - MIN[0])*scale - 60
						msg.position.z = -55
						pub.publish(msg)
						time.sleep(0.2)
						j+=1
						key = cv.waitKey(30)
						if key == ord('q') or key == 27:
							break
				"""
				
				scale=7
				x=220
				y=-140
				msg.position.x = x
				msg.position.y = y
				msg.position.z = -10
				pub.publish(msg)
				time.sleep(1)

				for string in strings:
					word="".join(ch.upper() for ch in string if(ch.isalnum() or ch=="/" or ch==" "))
					res=[]
					res[:] = word
					num=len(word)
					num=num/2
					y=-num*scale*4
					
					for letter in res:
						cord=coordinates(letter)
						print(letter)
						for k in cord:
							
							x_cord=x+k[1]*scale
							y_cord=y-k[0]*scale
							
							print(x_cord)
							print(y_cord)
							
							msg.position.x = x_cord
							msg.position.y = y_cord
							msg.position.z = k[2]
							pub.publish(msg)
							time.sleep(1)


						msg.position.x = x_cord
						msg.position.y = y_cord
						msg.position.z = -15
						pub.publish(msg)
						time.sleep(1)




					
						y-=np.min(cord,axis=0)[0]*scale
					x+=scale*4

				while(1):
					key = cv.waitKey(30)
					if key == ord('q') or key == 27:
						break
				
		except rospy.ROSInterruptException:
        		pass
		
		
