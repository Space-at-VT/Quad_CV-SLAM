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
showROIBox = True
drawKLT = True

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
# length of image diagonals in pixels (for motion rate calculations)
lenFrameDiag = np.sqrt(np.power(size[0],2) + np.power(size[1],2))
lenFrameDiag = np.int16(lenFrameDiag)
        
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
mFrame2Idx = mFrame1Idx + 100
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
        
# Detect keypoints within initial frame and within ROI
# initialize detector
orb = cv2.ORB_create()
# detect keypoints & extract BRIEF features
kp0, des0 = orb.detectAndCompute(hullBox,None)
kp0 = np.array(kp0)
# initialize point life tracker (#of succesful )
actPtLife = np.zeros([kp0.size,1],dtype=np.int16)

# log and translate keypoint coordinate frame to correct frame (cropped)
p0 = np.zeros([len(kp0),2],dtype=np.float32)
for ii in range(0,kp0.size):
    kp0[ii].pt = (kp0[ii].pt[0]+x,kp0[ii].pt[1]+y)
    p0[ii,:] = kp0[ii].pt
    
# show keypoints detected within ROI bounding box
if showROIBox:
    ROIMask = mFrameAbsDiff.copy()
    ROIMask = cv2.drawContours(ROIMask,[hullPolygon],-1,(255,255,255),2)
    ROIMask = cv2.rectangle(ROIMask,(x,y),(x+w,y+h),(255,255,255),2)
    ImPreview('Identified ROI',[ROIMask],0)
    
# Initialize frame analysis loop
# klt tracker
lk_params = dict( winSize = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# loop settings
mode = "init"
oldFrame = mFrame1Gray.copy()
bf = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)

# --- INITIALIZATION ---
# reset frame idx and capture to first frame
vid.set(cv2.CAP_PROP_POS_FRAMES,300)
# loop through all frames in video file
mask = np.zeros_like(oldFrame)
firstFrame = 1
rFrames = 5
for frameIdx in range(firstFrame,firstFrame + 100):
    # acquire new frame --> grayscale --> blur
    success, newFrameC = vid.read()
    if not(success):
        break
        
    newFrame = visfunctions.GrayBlur(newFrameC,5,5,3)
    # drawing mask
    if not(np.mod(frameIdx,30)) and (frameIdx != firstFrame) and drawKLT:
        mask = np.zeros_like(oldFrame)
    
    # propagate KLT tracker, identify good tracks
    p1, st, err = cv2.calcOpticalFlowPyrLK(oldFrame,newFrame,p0,None,**lk_params)
    stIdx = np.zeros([kp0.size,1],dtype = np.int16)
    
    # compare kp descriptors every xx frames to eliminate bad tracks
    if not(np.mod(frameIdx,rFrames)) and (frameIdx != firstFrame):
        # detect kp, compute des within ROI in new hull image
        newHull = newFrame[y:y+h,x:x+w]
        kp1, des1 = orb.detectAndCompute(newHull,None)
        kp1 = np.array(kp1)
        # adjust point coordinates from hull frame to image frame
        for ii in range(0,kp1.size):
            kp1[ii].pt = (kp1[ii].pt[0]+x,kp1[ii].pt[1]+y)
        # identify strongest matches between library des and current des
        matches = bf.match(des1,des0)
        matches = sorted(matches, key = lambda x:x.distance)
        # build orb matching indices and update succesful track index
        mTrainIdx = np.zeros([len(matches),1],dtype=np.int16)
        mQueryIdx = mTrainIdx.copy()
        for ii in range(0,len(matches)):
            mTrainIdx[ii,0] = matches[ii].trainIdx
            mQueryIdx[ii,0] = matches[ii].queryIdx
        stIdx[mTrainIdx] = 1
        stIdx[st[:,0]!=1] = 0
        # find new kp,des,pts in (1) to add to library (0)        
        nums = np.linspace(0,len(kp1)-1,len(kp1),dtype=np.int16)
        newPtIdx = np.setdiff1d(nums,mQueryIdx)
        # update library descriptor with matched descriptor
        des0[mTrainIdx] = des1[mQueryIdx]

    else:
        stIdx = st
       
    # select succesfully tracked & matched points
    p0 = p0[stIdx[:,0]==1]
    p1 = p1[stIdx[:,0]==1]
    
    # increment point life tracker for tracked & matched points
    actPtLife = actPtLife[stIdx[:,0]==1]
    actPtLife += 1
    
    # update kp0 and des0 to reflect succesful tracks
    kp0 = kp0[stIdx[:,0]==1]
    des0 = des0[stIdx[:,0]==1]
    
    # draw p0 to p1 point track lines    
    if drawKLT:
        mask = visfunctions.DrawTracks(mask,p0,p1)
        
    # add new pts,kp,des into library
    if not(np.mod(frameIdx,rFrames)) and (frameIdx != 300):
        newKp = kp1[newPtIdx]
        newKp = np.array(newKp)
        newDes = des1[newPtIdx]
        newPts = np.zeros([newKp.size,2],dtype=np.float32)
        for ii in range(0,newKp.size):
            newPts[ii,:] = newKp[ii].pt
        kp0 = np.append(kp0,newKp,axis = 0)
        des0 = np.append(des0,newDes,axis = 0)
        p1 = np.append(p1,newPts,axis = 0)
        ptLifeZeros = np.zeros([newKp.size,1],dtype = np.int16)
        actPtLife = np.append(actPtLife,ptLifeZeros,axis = 0)
                
    if drawKLT:
        # draw circles at new point locations
        if not(np.mod(frameIdx,rFrames)) and (frameIdx != 300):
            for ii in range(0,len(newPts)):
                cen = newPts[ii,0],newPts[ii,1]
                mask2 = np.zeros_like(mask)
                mask2 = cv2.circle(mask2,cen,3,[255,255,255],5)
            img = cv2.add(mask,mask2)
            img = cv2.add(img,newFrame)
        else:
            img = cv2.add(mask,newFrame)
        cv2.imshow('KLT',img)
        k = cv2.waitKey(31) & 0xff
        if k == 27:
            break
        
    # calculate average point motion for first x frames
        
    if frameIdx == firstFrame:
        gavg = np.zeros([10,1],dtype = np.float16)
        avgCount = 0
    if (frameIdx <= firstFrame + 10) and (frameIdx != firstFrame):
        ptAvgDistPerFrame = np.zeros([p0.size/2,1],dtype = np.float16)
        for ii in range(0,p0.size/2):
            dx = p1[ii,0] - kp0[ii].pt[0]
            dy = p1[ii,1] - kp0[ii].pt[1]
            df = actPtLife[ii,0] # frame delta: number of frames elapsed
            ptAvgDistPerFrame[ii] = np.sqrt(np.power(dx,2)+np.power(dy,2)) / df
        gavg[avgCount,0] = np.average(ptAvgDistPerFrame)
        avgCount += 1
        
            
    # update old frame and old points
    oldFrame = newFrame.copy()
    p0 = p1.copy()
    
# end of entire program
print .01 * lenFrameDiag / np.average(gavg)
print "frames"
cv2.destroyAllWindows()
vid.release()    
    
# --- EXPERIMENTAL ---
if useExperimental:
    # experimental code goes here
    abcd = 1

    #img1 = cv2.imread("Images\demoimage2.jpg")  
    #fast = cv2.FastFeatureDetector_create()
    #kpa = orb.detect(img1,None)
    #kptx,desb = orb.compute(img1,kpa)
    #test = np.array(kpa)
    #kpt,desa = orb.compute(img1,test)

    #p1, st, err = cv2.calcOpticalFlowPyrLK(oldFrame,newFrame,keypointCoord,None,**lk_params)
    #good_new = p1[st[:,0]==1]
    #good_old = keypointCoord[st[:,0]==1]
    #mask = visfunctions.DrawTracks(oldFrame,good_old,good_new)
    #ImPreview([mask],0)  

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







