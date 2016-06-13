# -*- coding: utf-8 -*-
"""
Created on Mon Feb 15 12:40:34 2016

@author: Loren
"""

# --- PROGRAM SETUP ---
# Monocular SLAM Pose Estimation for Satellite ProxOps

# Modules
import cv2
import numpy as np
import visfunctions
from visfunctions import ImPreview as ImPreview
import bigfunctions
#from matplotlib import pyplot as plt

# Global Variables
global kp0, kp1, des0, des1, p0, p1, actPtLife
global lk_params, firstFrame
global orb, bf

# Program Settings
previewVideo = False
showROIBox = True
drawKLT = True

# Select Video File
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
width = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)) 
# length of image diagonals in pixels (for motion rate calculations)
lenFrameDiag = np.sqrt(np.power(size[0],2) + np.power(size[1],2))
lenFrameDiag = np.int16(lenFrameDiag)

# Select camera calibration matrix, K
K = np.array([[width,0,width/2],[0,width,height/2],[0,0,1]])
        
# preview video
if previewVideo:
    cv2.namedWindow(vidTitle)
success, frame = vid.read()
while success and cv2.waitKey(31) == -1 and previewVideo:
    cv2.imshow(vidTitle,frame)
    success, frame = vid.read()
    if not(success):
        cv2.destroyWindow(vidTitle)
        
# --- MAIN LOOP ---
mode = "preinit"
firstFrame = np.int32(6*30)
frameIdx = firstFrame
frameIdxMaximum = 300

while frameIdx < frameIdxMaximum:
    # Loop through program modes
    if mode == "preinit":
        if not("hullBox" in locals()):
            # Find initial region of interest, calculate bounding box
            if frameIdx == 0:
                mFrame1Idx = frameIdx
                mFrame2Idx = mFrame1Idx + 30
            elif frameIdx == mFrame1Idx:
                success, mFrame1 = vid.read()
                if not(success):
                    break
                frameIdx += frameIdx
            elif frameIdx == mFrame2Idx:
                success, mFrame2 = vid.read()
                if not(success):
                    break
                frameIdx += frameIdx
                hullBox,x,y,w,h = bigfunctions.LocateROI(mFrame1,mFrame2)
                roiVec = np.array([x,y,w,h])
        elif "hullbox" in locals():
            if not("orb" in locals()):
                # initialize ORB detector, detect library kp and feats
                orb = cv2.ORB_create()
                bf = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
                kp0, des0 = orb.detectAndCompute(hullBox,None)
                kp0 = np.array(kp0)
                # initialize point life tracker (#of succesful )
                actPtLife = np.zeros([kp0.size,1],dtype=np.int16)
                # intialize KLT tracker
                lk_params = dict( winSize = (15,15), 
                                  maxLevel = 2, 
                                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
                # log and translate keypoint coordinate frame to correct frame (cropped)
                p0 = np.zeros([len(kp0),2],dtype=np.float32)
                for ii in range(0,kp0.size):
                    kp0[ii].pt = (kp0[ii].pt[0]+x,kp0[ii].pt[1]+y)
                    p0[ii,:] = kp0[ii].pt
                oldFrame = mFrame1.copy()
                oldFrame = visfunctions.GrayBlur(oldFrame,5,5,3)
                mask = np.zeros_like(oldFrame)
                rFrames = 5 # rate to run ORB feature matching
                
            # loop thru frames to estimate pixel motion
            loopDummy = frameIdx
            for ii in range(loopDummy,loopDummy+30):
                success, newFrameColor = vid.read()
                if not(success):
                    break
                frameIdx += 1
                newFrame = visfunctions.GrayBlur(newFrameColor,5,5,3)
                oldFrame, avgMotion = bigfunctions.TrackMatch(oldFrame,
                                                   newFrame,
                                                   frameIdx,
                                                   rFrames,
                                                   roiVec,
                                                   "motion",1)
            avgFrameMotion = .01 * lenFrameDiag / np.average(avgMotion)
            mode = "init" 
        # loop through first xx frames
        # calculate motion & initialization thresholds
        # set motion counter = 0
        # set mode = init
    elif mode == "init":
        test = 2
        # loop through frames until motion counter > init thresh
        # calculate F,E,P1,P2
        # triangulate points to 3D
        # set motion counter = 0
        # set mode = pnp
    elif mode == "pnp":
        test = 3
        # loop through frames until motion counter > motion thresh
        # pnp pose estimation function
        # triangulation function
        # set motion counter = 0
        # if # poses == 3, then SBA    

# --- END PROGRAM ---
cv2.destroyAllWindows()
vid.release()    






