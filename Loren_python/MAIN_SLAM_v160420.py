# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 16:15:14 2016

@author: Loren
"""

# --- PROGRAM SETUP ---
# Monocular SLAM Pose Estimation for Satellite ProxOps

# Modules
import cv2
import numpy as np
#from matplotlib import pyplot as plt
# for sorting
import heapq
# Custom Functions
import cvtools
# Custom Classes
from visclass import VidFile
from visclass import PointCloud

# Program Settings
previewVideo = False
showROIBox = False # doesn't work yet!
drawKLT = True

# Select Video File
#videoFilepath = "..\Videos\data050.avi"
#videoFilepath = "..\Videos\data051.avi"
#videoFilepath = "..\Videos\data060.avi"
#videoFilepath = "..\Videos\ShuttleOrbit.wmv"
#videoFilepath = "..\Videos\Vehicle2.wmv"
videoFilepath = "..\Videos\TAMUHomer.mp4"
vidTitle = videoFilepath.rsplit('\\')[1]

# Create primary instance of VidFile
vid = VidFile(videoFilepath,vidTitle)

if previewVideo:
    cv2.namedWindow(vid.title)
success, frame = vid.file.read()
while success and cv2.waitKey(31) == -1 and previewVideo:
    cv2.imshow(vid.title,frame)
    success, frame = vid.file.read()
    if not(success):
        cv2.destroyWindow(vid.title)
        
# --- MAIN LOOP ---
mode = "preinit"
firstFrame = np.int32(500)
vid.file.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, firstFrame)
frameIdx = firstFrame
frameIdxMaximum = 800
rFrames = 5
mFrame1Idx = firstFrame
mFrame2Idx = mFrame1Idx + 30
while frameIdx < frameIdxMaximum:
    # loop through program modes
    if mode == "preinit":
        if not("hullBox" in locals()):
            # Find initial region of interest, calc bounding box
            if frameIdx == mFrame1Idx:
                success, frame = vid.file.read()
                mFrame1 = np.copy(frame)
                if not(success):
                    vid.close_out()
                    break
                frameIdx += 1
            elif frameIdx == mFrame2Idx:
                success, frame = vid.file.read()
                mFrame2 = np.copy(frame)
                if not(success):
                    vid.close_out()
                    break
                frameIdx += 1
                hullBox,x,y,w,h = cvtools.LocateROI(mFrame1,mFrame2)
                roiVec = np.array([x,y,w,h])
                # initialize orb and bf matching objects
                orb = cv2.ORB_create()
                bf = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck = True)
                # detect initial kp, des and initialized point cloud object
                kp0, des0 = orb.detectAndCompute(hullBox,None)
                kp0 = np.array(kp0)
                cloud = PointCloud(kp0,des0,roiVec)
                # initialize parameters for point propagation
                oldFrame = np.copy(mFrame1)
                oldFrame = cvtools.GrayBlur(oldFrame,5,5,3)
                mask = np.zeros_like(oldFrame)
                # initialize KLT tracker
                lkParams = dict( winSize = (15,15),
                                maxLevel = 2,
                                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
            else:
                success, frame = vid.file.read()
                if not(success):
                    vid.close_out()
                    break
                frameIdx += 1
            
            # show image
            cv2.imshow(vid.title,frame)
            k = cv2.waitKey(31) & 0xff
            if k == 27:
                break
                
        elif "cloud" in locals():
            # loop thru frames to estimate pixel motion
            loopDummy = np.copy(frameIdx)
            for jj in range(loopDummy,loopDummy + 30):
                # get new frame
                success, frame = vid.file.read()
                newFrameColor = np.copy(frame)
                if not(success):
                    break
                newFrame = cvtools.GrayBlur(newFrameColor,5,5,3)
                # propagate point motion, tracking and matching
                if np.mod(frameIdx,rFrames) == 0 and (frameIdx != firstFrame):
                    trackMode = "match"
                else:
                    trackMode = "track"
                p0,p1 = cvtools.TrackMatch(cloud,oldFrame,newFrame,lkParams,orb,bf,trackMode)
                oldFrame = np.copy(newFrame)
                # calculate the average motion
                if frameIdx == loopDummy:
                    frameAvg = np.zeros([30,1],dtype = np.float16)
                    avgCount = 0
                else:
                    ptAvgDistPerFrame = np.zeros([p0.size/2,1],dtype = np.float16)
                    dfVec = cloud.actPtLife[cloud.recentTrackIdx[:,0]==1,0]
                    for ii in range(0,p0.size/2):
                        dx = p1[ii,0] - p0[ii,0]
                        dy = p1[ii,1] - p0[ii,1]
                        df = dfVec[ii]
                        ptAvgDistPerFrame[ii] = np.sqrt(np.power(dx,2)+np.power(dy,2))/df
                    ptAvgDistPerFrame = ptAvgDistPerFrame[ptAvgDistPerFrame < 10000]
                    frameAvg[avgCount,0] = np.average(ptAvgDistPerFrame)
                    avgCount += 1
                cv2.imshow(vid.title,frame)
                k = cv2.waitKey(31) & 0xff
                if k == 27:
                    break
                frameIdx +=1
            avgFrameMotion = 0.01 * vid.lenDiag / np.average(frameAvg)
            initThresh = frameIdx + int(round(avgFrameMotion)) - 1
            mode = "init"
            motionCounter = 0
            frozenPts = p1
            
    # NOT COMPLETELY FINISHED BEYOND THIS POINT
    # THE ABOVE IS NOT OPTIMIZED EITHER (and missing some functionality)
    
    elif mode == "init":
        # get new frame, continue point tracking
        success, frame = vid.file.read()
        newFrameColor = np.copy(frame)
        if not(success):
            break
        newFrame = cvtools.GrayBlur(newFrameColor,5,5,3)
        # propagate point motion, tracking and matching
        if np.mod(frameIdx,rFrames) == 0 and (frameIdx != firstFrame):
            trackMode = "match"
        else:
            trackMode = "track"
        p0,p1 = cvtools.TrackMatch(cloud,oldFrame,newFrame,lkParams,orb,bf,trackMode)
        oldFrame = np.copy(newFrame)
        
        cv2.imshow(vid.title,frame)
        k = cv2.waitKey(31) & 0xff
        if k == 27:
            break
        
        frameIdx += 1
        motionCounter += 1
        
        if frameIdx >= initThresh:
            limit = int(round(avgFrameMotion))
            while sum(cloud.actPtLife[:,0]>limit) < 8:
                limit -= 1
            curIdx1 = np.asarray(heapq.nlargest(8,cloud.actPtLife))
            matchIdx = -1 * np.ones([curIdx1.size,1],dtype = np.int16)
            curIdx1 = np.unique(curIdx1)
            curIdx1[::] = curIdx1[::-1]
            jj = 0
            for ii in range(0,curIdx1.size):
                aa = np.where(cloud.actPtLife == curIdx1[ii])
                aa = aa[0]
                while (jj + aa.size) > matchIdx.size:
                    # wont fit unless shrunken
                    aa = np.delete(aa,aa.size-1)
                # will fit
                matchIdx[jj:jj+aa.size,0] = aa
                jj = jj + aa.size
            
            curPts = p1[matchIdx,:]
            oldPts = frozenPts[matchIdx,:]
            # get Fundamental Matrix (estimate... may be ill-conditioned)
            fundaMat = cv2.findFundamentalMat(oldPts,curPts,cv2.FM_8POINT)
            # calculate first pose estimate (pose at start of init)
            zeroVec = np.array([0,0,0])
            zeroVec.shape = (3,1)
            P0 = np.dot(vid.K,np.concatenate((np.identity(3),zeroVec),1))
            mode = "pnp"
            
    elif mode == "pnp":
        frameIdx += 1
        # 
# Close video window and release video file memory
vid.close_out()