# -*- coding: utf-8 -*-
"""
Created on Mon Feb 15 12:40:34 2016

@author: Loren
"""

# --- PROGRAM SETUP & CONTROLS ---
# Monocular SLAM Pose Estimation for Satellite ProxOps

# Import modules
import cv2
import numpy as np
import visfunctions
from visfunctions import ImPreview as ImPreview
#from matplotlib import pyplot as plt

# Configure program settings
useExperimental = False
previewVideo = False
previewFrames = True

# Select video file
#videoFilepath = "Videos\data050.avi"
#videoFilepath = "Videos\data051.avi"
#videoFilepath = "Videos\data060.avi"
#videoFilepath = "Videos\ShuttleOrbit.wmv"
#videoFilepath = "Videos\Vehicle2.wmv"
videoFilepath = "Videos\TAMUHomer.mp4"
vidTitle = videoFilepath.rsplit('\\')[1]

# load video, get info
vid = cv2.VideoCapture(videoFilepath)
fps = vid.get(cv2.CAP_PROP_FPS)
size = (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        
# preview video
if previewVideo:
    cv2.namedWindow(vidTitle)
success, frame = vid.read()
while success and cv2.waitKey(31) == -1 and previewVideo:
    cv2.imshow(vidTitle,frame)
    success, frame = vid.read()
cv2.destroyWindow(vidTitle)

# --- PRE-INITIALIZATION ---
mode = "preinit"
# Identify convex ROI polygon
# select motion frames
mFrame1Idx = 300;
mFrame2Idx = mFrame1Idx + 30
vid.set(cv2.CAP_PROP_POS_FRAMES, mFrame1Idx)
success, mFrame1 = vid.read()
vid.set(cv2.CAP_PROP_POS_FRAMES, mFrame2Idx)
success, mFrame2 = vid.read()

# convert to grayscale and convolve w/ Gaussian kernel (blur)
mFrame1Gray = visfunctions.GrayBlur(mFrame1,5,5,3)
mFrame2Gray = visfunctions.GrayBlur(mFrame2,5,5,3)

# estimate region of interest
hullPolygon, mFrameAbsDiff = visfunctions.IdHullOne(mFrame1Gray,mFrame2Gray)
x,y,w,h = cv2.boundingRect(hullPolygon)
hullBox = mFrame1Gray[y:y+h,x:x+w]

# preview selected frames and convex polygon
if previewFrames:
    cv2.drawContours(mFrameAbsDiff,[hullPolygon],-1,(255,255,255),3)
    ImPreview("Preview",[mFrame1,mFrame2,mFrameAbsDiff],0)
        
# Detect keypoints within initial frame and within ROI
# initialize detector
orb = cv2.ORB_create()
# detect keypoints & extract BRIEF features
kp0, des0 = orb.detectAndCompute(hullBox,None)
# log and translate keypoint coordinate frame to correct frame (cropped)
keypointCoord = np.zeros([len(kp0),2],dtype=np.float32)
for ii in range(0,len(kp0)):
    kp0[ii].pt = (kp0[ii].pt[0]+x,kp0[ii].pt[1]+y)
    keypointCoord[ii,:] = kp0[ii].pt
    
# show keypoints detected within ROI bounding box
if previewFrames:
    roiBoxKp0 = cv2.drawKeypoints(hullBox,kp0,mFrame1,color=(0,255,0),flags=0)
    ImPreview("Preview",[roiBoxKp0],0)
    
# Initialize frame analysis loop
# klt tracker
lk_params = dict( winSize = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# loop settings
mode = "init"
oldFrame = mFrame1Gray.copy()

# --- INITIALIZATION ---
# reset frame idx and capture to first frame
vid.set(cv2.CAP_PROP_POS_FRAMES,300)
# loop through all frames in video file
mask = np.zeros_like(oldFrame)
for frameIdx in range(700,1000):
    # acquire new frame --> grayscale --> blur
    success, newFrameC = vid.read()
    newFrame = visfunctions.GrayBlur(newFrameC,5,5,3)
    
    # propagate KLT tracker
    p1, st, err = cv2.calcOpticalFlowPyrLK(oldFrame,newFrame,keypointCoord,None,**lk_params)
    good_new = p1[st[:,0]==1]
    good_old = keypointCoord[st[:,0]==1]
    mask = visfunctions.DrawTracks(mask,good_old,good_new)
    img = cv2.add(mask,newFrame)
    cv2.imshow('KLT',img)
    k = cv2.waitKey(31) & 0xff
    if k == 27:
        break
    oldFrame = newFrame.copy()
    keypointCoord = p1.copy()
    
    # 
    
    
    
    
    
    
cv2.destroyAllWindows()
vid.release
    
    
    
    
    
    
    
    
    
    
    
    



# --- EXPERIMENTAL ---
if useExperimental:
    # experimental code goes here
    p1, st, err = cv2.calcOpticalFlowPyrLK(oldFrame,newFrame,keypointCoord,None,**lk_params)
    good_new = p1[st[:,0]==1]
    good_old = keypointCoord[st[:,0]==1]
    mask = visfunctions.DrawTracks(oldFrame,good_old,good_new)
    ImPreview([mask],0)
        

    # need to translate keypoint coordinates derived from motionHull image to
    # global image coordinate system
    #mask1 = mFrame1Gray.copy()
    #mask2 = hullBox.copy()
    #cv2.rectangle(mask1,(x,y),(x+w,y+h),[255,255,255],5)
    #ImPreview([mask1],0)
    #testPt = (int(kp0[0].pt[0]),int(kp0[0].pt[1]))
    #testPt = (testPt[0]+x,testPt[1]+y)
    #testPt = (int(kp0[0].pt[0]),int(kp0[0].pt[1]))
    #cv2.circle(mask1,testPt,10,[255,255,255],5)
    #cv2.circle(mask2,testPt,10,[255,255,255],5)
    #ImPreview([mask1,mask2],0)









    #roiImg1 = mFrame2Gray[y:y+h,x:x+w]
    #kp1, des1 = orb.detectAndCompute(roiImg1,None)
    #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
    #matches = bf.match(des0,des1)
    #matches = sorted(matches, key = lambda x:x.distance)
    #img2 = cv2.drawMatches(roiImg0,kp0,roiImg1,kp1,matches[:30],mFrame2,flags = 2)
    #ImPreview([img2],0)
    test = 1







