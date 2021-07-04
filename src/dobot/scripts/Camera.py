from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np

import rospy
from geometry_msgs.msg import Pose
import time
from scipy.interpolate import splprep, splev







max_value = 255
max_value_H = 360//2
low_H = 56
low_S = 56
low_V = 11
high_H = 112
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


parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
cap = cv.VideoCapture(args.camera)
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
print("Press q/esc to quit and y to start writing")
while True:
    
	ret, frame = cap.read()
	if frame is None:
		break
	frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
	frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))


	cv.imshow(window_capture_name, frame)
	cv.imshow(window_detection_name, frame_threshold)

	
	im2, cntrs, hierarchy = cv.findContours(frame_threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
	cntrs=np.array(cntrs)

	contoured=cv.drawContours(frame, cntrs, -1, (0,255,0), 1)
	cv.imshow(window_contour_name, contoured)
	
	'''
	(thresh, binRed) = cv.threshold(frame_threshold, 250, 255, cv.THRESH_BINARY)

	_, Rcontours, hier_r = cv.findContours(binRed,cv.RETR_CCOMP,cv.CHAIN_APPROX_SIMPLE)
	r_areas = [cv.contourArea(c) for c in Rcontours]
	max_rarea = np.max(r_areas)
	CntExternalMask = np.ones(binRed.shape[:2], dtype="uint8") * 255

	for c in Rcontours:
		if(( cv.contourArea(c) > max_rarea * 0.70) and (cv.contourArea(c)< max_rarea)):
			img=cv.drawContours(CntExternalMask,[c],-1,0,1)

	cv.imshow('Image', img)
	'''
	key = cv.waitKey(30)
	if key == ord('q') or key == 27:
		break
	elif key==ord('y'):
		try:
			
			pub = rospy.Publisher('geometry_pose', Pose, queue_size=10)
			rospy.init_node('talker', anonymous=True)
			rate = rospy.Rate(2000) # 10hz
		
			MIN_x=[np.min(cnt,axis=0)[0][0] for cnt in cntrs]
			MIN_y=[np.min(cnt,axis=0)[0][1] for cnt in cntrs]
	
			MAX_x=[np.max(cnt,axis=0)[0][0] for cnt in cntrs]
			MAX_y=[np.max(cnt,axis=0)[0][1] for cnt in cntrs]

			MIN=[np.min(MIN_x), np.min(MIN_y)]
			MAX=[np.max(MAX_x), np.max(MAX_y)]

			"""
			#Starting point calibration
			msg = Pose()

			msg.position.x = 310 #starting point 170 (140 max)
			msg.position.y = 90 #starting point -50 (140 max)
			msg.position.z = -50

			pub.publish(msg)
			time.sleep(0.1)
			"""
			total_x=200
			total_y=200

			scale=0.5
			total_x= (MAX[1] - MIN[1])*scale + 200
			total_y= (MAX[0] - MIN[0])*scale 

			while(total_x>300 or total_y>140):
				scale-=0.01
				total_x= (MAX[1] - MIN[1])*(scale-0.1) + 200
				total_y= (MAX[0] - MIN[0])*(scale+0.1)
				
			msg = Pose()	
			msg.position.x = 200 
			msg.position.y = 0 
			msg.position.z = -15
			pub.publish(msg)
			time.sleep(0.2)
	
		
			for cnt in cntrs:
				j=0
				if(len(cnt)<=20):
					continue
				print("Length: ",len(cnt))
				'''
				x,y = cnt.T
				# Convert from numpy arrays to normal arrays
				x = x.tolist()[0]
				y = y.tolist()[0]
				# https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.interpolate.splprep.html
				tck, u = splprep([x,y], u=None, s=1.0, per=1)
				# https://docs.scipy.org/doc/numpy-1.10.1/reference/generated/numpy.linspace.html
				u_new = np.linspace(u.min(), u.max(), 25) 
				# https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.interpolate.splev.html
				x_new, y_new = splev(u_new, tck, der=0)
				# Convert it back to numpy format for opencv to be able to display it
				res_array = [[[int(i[0]), int(i[1])]] for i in zip(x_new,y_new)]
				smoothened=np.asarray(res_array, dtype=np.int32)
				'''
				
				for cnt1 in cnt:
					print("x:",(cnt1[0][1] - MIN[1])*scale + 200)
					print("y:", (cnt1[0][0] - MIN[0])*scale - MIN[0] )

					if(j==0):
						msg.position.x = (cnt1[0][1] - MIN[1])*scale + 200 
						msg.position.y = (cnt1[0][0] - MIN[0])*scale 
						msg.position.z = -15
						pub.publish(msg)
						time.sleep(1)

					
					msg.position.x = (cnt1[0][1] - MIN[1])*scale + 200
					msg.position.y = (cnt1[0][0] - MIN[0])*scale 
					msg.position.z = -53
					pub.publish(msg)
					time.sleep(0.2)
					j+=1
					key = cv.waitKey(30)
					if key == ord('q') or key == 27:
						break

			msg.position.x = (cnt1[0][1] - MIN[1])*scale + 200
			msg.position.y = (cnt1[0][0] - MIN[0])*scale 
			msg.position.z = -15
			pub.publish(msg)
			time.sleep(0.2)
			
		except rospy.ROSInterruptException:
        		pass
		
		
