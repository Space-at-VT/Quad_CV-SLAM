# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 10:53:07 2016

@author: Loren
"""
# Goal of the program is to (1) subtract the background from ROI
# and (2) to optionally plot the ROI on the image frames
# this will be called before feature detection
# this function should return an array of 

# import modules
import cv2
import numpy as np
#import matplotlib.pyplot as plt

# set adjustable parameters
playVideo = 1
#videoFilepath = "Videos\data050.avi"
#videoFilepath = "Videos\data051.avi"
#videoFilepath = "Videos\data060.avi"
videoFilepath = "Videos\ShuttleOrbit.wmv"
#videoFilepath = "Videos\Vehicle2.wmv"
#videoFilepath = "Videos\webcam1.mp4"
windowTitle = videoFilepath.rsplit('\\')[1]

# load video, get info
videoCapture = cv2.VideoCapture(videoFilepath)
fps = videoCapture.get(cv2.CAP_PROP_FPS)
size = (int(videoCapture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(videoCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)))

# play video
if playVideo == 1:
    cv2.namedWindow(windowTitle)
success, frame = videoCapture.read()
while success and cv2.waitKey(31) == -1 and playVideo == 1:
    cv2.imshow(windowTitle,frame)
    success, frame = videoCapture.read()
cv2.destroyWindow(windowTitle)

# identify motion frames
videoCapture.set(cv2.CAP_PROP_POS_FRAMES, 100)
success, mFrame1 = videoCapture.read()
videoCapture.set(cv2.CAP_PROP_POS_FRAMES, 130)
success, mFrame2 = videoCapture.read()

cv2.imshow('window',mFrame1)
cv2.waitKey(0)
cv2.imshow('window',mFrame2)
cv2.waitKey(0)
#cv2.destroyWindow('window')

# normalized image difference
mFrame1 = cv2.cvtColor(mFrame1, cv2.COLOR_BGR2GRAY)
mFrame2 = cv2.cvtColor(mFrame2, cv2.COLOR_BGR2GRAY)
mFrame1 = cv2.GaussianBlur(mFrame1,(5,5),3)
mFrame2 = cv2.GaussianBlur(mFrame2,(5,5),3)

frameAvg = (np.double(mFrame1) + np.double(mFrame2))/2
frameDiff1 = cv2.absdiff(mFrame1,mFrame2)
retval, frameDiff2 = cv2.threshold(frameDiff1, 25, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#frameDiff3 = cv2.dilate(frameDiff2, None, iterations = 1)
frameDiff4 = cv2.erode(frameDiff2, None, iterations = 2)
#im, contours, _ = cv2.findContours(frameDiff4.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#contours2 = sorted(contours,key=cv2.contourArea,reverse=True)[0]
pixelPoints = cv2.findNonZero(frameDiff4)
pixelPoints2 = pixelPoints[:,0]
#plt.plot(pixelPoints2[:,0],pixelPoints2 [:,1])
#plt.show()
hull = cv2.convexHull(pixelPoints)

cv2.drawContours(frameDiff1,[hull],-1,(255,255,255),5)

cv2.imshow('window',frameDiff1)
cv2.waitKey(0)
cv2.destroyWindow('window')













