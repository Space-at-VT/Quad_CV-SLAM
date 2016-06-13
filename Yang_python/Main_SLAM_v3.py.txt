"""
Created on Wed Jun 1 2016

@author: Yang
"""
#!python3


#module
import numpy as np
import cv2

#constants
import modes


#custom classes
from convexHull import ConvexHull
from KLTtracker import KLTtracker
from pointCloud import PointCloud
from reconstructor import PointReconstructor

#parameters
showROIBox = True
showKeypoints = True
showTracks = True

ROIUpdateFrames = 10
MatchUpdateFrames = 5

#set up mode
mode = modes.PRE_INIT

#videoInput
cap = cv2.VideoCapture("data051.avi")

#object initialization
convexHull = ConvexHull()
bf = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck = True)
orb = cv2.ORB_create()
tracker = None
rect = None
p0 = None
p1 = None

cv2.ocl.setUseOpenCL(False)

#frame counter
counter = 0

#main loop
while(cap.isOpened()):
    ret, frame = cap.read()
    if(type(frame)!=np.ndarray):
        break

    #updates ROI every certain number of frames
    if(counter%ROIUpdateFrames==0 and counter!=0):
        hull,rect = convexHull.boundingRect(frame)

    if(mode!=modes.PRE_INIT):
        img = frame[rect[1]:rect[1]+rect[3],rect[0]:rect[0]+rect[2]]

        #refreshes points being tracked every certain number of frames
        if(counter%MatchUpdateFrames==0 and counter!=0):
            p0,p1 = tracker.match(frame,orb,bf)
        else:
            p0,p1 = tracker.track(frame)
        kp, des = orb.detectAndCompute(img, None)
    
    if(mode==modes.PRE_INIT):
        #gets ROI
        hull,rect = convexHull.boundingRect(frame)
        img = frame[rect[1]:rect[1]+rect[3],rect[0]:rect[0]+rect[2]]
        kp, des = orb.detectAndCompute(img, None)
        
        if(counter==1):
            mode = modes.INIT
            tracker = KLTtracker(frame,kp,des,rect)

    elif(mode==modes.INIT):
        pass

    #draws point tracks
    if(showTracks):
        if(p0!=None and p1!=None):
            for i in range(len(p0)):
                frame = cv2.line(frame,tuple(p0[i]),tuple(p1[i]),(0,255,0),2)

    #draws keypoints
    if(showKeypoints):
        img = frame[rect[1]:rect[1]+rect[3],rect[0]:rect[0]+rect[2]]
        if(img.size!=0):
            img = cv2.drawKeypoints(img, kp, img, color=(255,0,0))

    #draws ROI            
    if(showROIBox):        
        frame = cv2.drawContours(frame,[hull],-1,(0,0,255),1)
        frame = cv2.rectangle(frame,(rect[0],rect[1]),(rect[0]+rect[2],rect[1]+rect[3]),(255,0,0),1)

    cv2.imshow('frame',frame)
    k = cv2.waitKey(16) & 0xff
    if k == ord("q"):
        break

    counter+=1
cap.release()
cv2.destroyAllWindows()
