# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 10:53:07 2016

@author: Loren
"""
# import modules
import cv2
import numpy as np

def ImPreview(winName,frames,wait):
    # frames is a list of cv2.VideoCapture objects
    for ii in range(len(frames)):
        cv2.imshow(winName,frames[ii])
        cv2.waitKey(wait)
    cv2.destroyWindow(winName)
    
    return None

def GrayBlur(frame,k1,k2,sig):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame,(k1,k2),sig)
    
    return frame
    
def IdHullOne(frame1,frame2):
    # calculate frame difference and erode background
    frameAbsDiff = cv2.absdiff(frame1,frame2)
    retval, threshMask = cv2.threshold(frameAbsDiff,25,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #threshMask = cv2.dilate(threshMask, None, iterations = 4)    
    erodeMask = cv2.erode(threshMask, None, iterations = 2)
    # locate remaining pixels and calculate encompassing convex polygon
    whitePixels = cv2.findNonZero(erodeMask)
    whitePixels = whitePixels[:,0]
    hullPolygon = cv2.convexHull(whitePixels)
    hullPolygon = hullPolygon[:,0]
    
    return hullPolygon, frameAbsDiff
        
def DrawTracks(mask,pts0,pts1):
    # draws lines going from (x,y) in pts0 to (x,y) in pts1
    # pts0 and and pts1 must both be shape(n,2)
    # lines drawn on a mask of same shape as img
    numPts = len(pts0)
    color = np.random.randint(0,255,(numPts,3))
    for ii in range(numPts):
        a = pts0[ii,0],pts0[ii,1]
        b = pts1[ii,0],pts0[ii,1]
        mask = cv2.line(mask,a,b,color[ii].tolist(),2)
        
    return mask
    