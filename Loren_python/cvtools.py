# -*- coding: utf-8 -*-
"""
Created on Thu Mar 24 18:06:54 2016

@author: Loren
"""

# global kp0, des0, p0, lk_params

# Modules
import cv2
import numpy as np
import visfxns

def GrayBlur(frame,k1,k2,sig):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame,(k1,k2),sig)
    
    return frame

def LocateROI(frame1,frame2):
    # convert frames to gray & blur
    mFrame1Gray = GrayBlur(frame1,5,5,3)
    mFrame2Gray = GrayBlur(frame2,5,5,3)
    
    # calculate frame difference and erode background
    frameAbsDiff = cv2.absdiff(mFrame1Gray,mFrame2Gray)
    retval, threshMask = cv2.threshold(frameAbsDiff,25,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #threshMask = cv2.dilate(threshMask, None, iterations = 4)    
    erodeMask = cv2.erode(threshMask, None, iterations = 2)
    
    # locate remaining pixels and calculate encompassing convex polygon
    whitePixels = cv2.findNonZero(erodeMask)
    whitePixels = whitePixels[:,0]
    hullPolygon = cv2.convexHull(whitePixels)
    hullPolygon = hullPolygon[:,0]
    x,y,w,h = cv2.boundingRect(hullPolygon)
    hullBox = mFrame1Gray[y:y+h,x:x+w]
    
    return hullBox,x,y,w,h
    
def TrackMatch(cloud,oldFrame,newFrame,lkParams,orb,bf,mode):
    # propagate points to new frame using KLT tracker
    p1, st, err = cv2.calcOpticalFlowPyrLK(oldFrame,newFrame,
                                           cloud.p0,None,**lkParams)
    stIdx = np.zeros([cloud.kp0.size,1],dtype = np.int16)
    
    if mode == "track":
        stIdx = st
    elif mode == "match":
       # detect kp, compute des within ROI in new hull image
        newHull = newFrame[cloud.y:cloud.y+cloud.h,cloud.x:cloud.x+cloud.w]
        kp1, des1 = orb.detectAndCompute(newHull,None)
        kp1 = np.array(kp1)
        # adjust point coordinates from hull frame to image frame
        for ii in range(0,kp1.size):
            kp1[ii].pt = (kp1[ii].pt[0]+cloud.x,kp1[ii].pt[1]+cloud.y)
        # identify strongest matches between library des and current des
        matches = bf.match(des1,cloud.des0)
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
        cloud.des0[mTrainIdx] = des1[mQueryIdx]
     
    # select succesfully tracked & matched points
    p0 = cloud.p0.copy()
    p0 = p0[stIdx[:,0]==1]
    p1 = p1[stIdx[:,0]==1]
        
    # increment point life tracker for tracked & matched points 
    cloud.increment_life(stIdx)
    # update kp0 and des0 to reflect succesful tracks
    cloud.kp0 = cloud.kp0[stIdx[:,0] == 1]
    cloud.des0 = cloud.des0[stIdx[:,0] == 1]
    cloud.p0 = p1.copy()
    # update the most recent tracking index log
    cloud.recentTrackIdx = stIdx
    
    if mode == "match":
        # append newly found des, kp to library
        cloud.update_lib(kp1,des1,newPtIdx)
            
    return p0,p1
   
    
    
def TrackMatchOld(oldFrame,newFrame,firstFrame,frameIdx,rFrames,
               roiVec,kp0,des0,p0,actPtLife,
               orb,bf,mode,drawKLT):
    # oldFrame: previous image
    # newFrame: current image
    # frameIdx: integer location of current frame within all frames
    # rFrames: frame rate to run ORB feature matching (every rFrames do...)
    # roiVec: offset of Hullbox within frames
    # mode: if "motion" calculate average pixel motion
    # drawKLT: 1 | 0 to draw tracks
    # kp0: previous image keypoints
    # kp1: current image keypoints
    # des0: previous image descriptors
    # des1: current image descriptors
    # p0: previous image identified image coordinates of kp0   
    # p0: previous image identified image coordinates of kp0
    # actPtLife: vector tracking # of frames points have existed
    # lk_params: settings for lk tracker
    # firstFrame: index of first frame in video (usually 0)
    # orb: oriented and rotated brief object
    # bf: brute-force matcher object           
    
    # parse inputs
    x = roiVec(0)
    y = roiVec(1)
    w = roiVec(2)
    h = roiVec(3)
    
    # propagate KLT tracker, identify good tracks
    p1, st, err = cv2.calcOpticalFlowPyrLK(oldFrame,newFrame,p0,None,**lk_params)
    stIdx = np.zeros([kp0.size,1],dtype = np.int16)
    
    # drawing mask
    if drawKLT:
        mask = np.zeros_like(oldFrame)
    
    # compare kp descriptors every rFrames frames to eliminate bad tracks
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
    
    # increment point life tracker for tracked & matched points
    actPtLife = actPtLife[stIdx[:,0]==1]
    actPtLife += 1
    
    # update kp0 and des0 to reflect succesful tracks
    kp0 = kp0[stIdx[:,0]==1]
    des0 = des0[stIdx[:,0]==1]
        
    # draw p0 to p1 point track lines    
    if drawKLT:
        mask = visfxns.DrawTracks(mask,p0,p1)
        
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
        #if k == 27:
            #break
        
    if mode == "motion":
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
    else:
        avgCount = 0
    
    # update old frame and old points
    oldFrame = newFrame.copy()
    p0 = p1.copy()
    
    return oldFrame, kp0, des0, p0, actPtLife, gavg
    
    
    
    
    
    
    
    
    
    
    
    
    