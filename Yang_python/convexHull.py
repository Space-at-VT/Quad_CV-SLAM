"""
Created on Wed Jun 1 2016

@author: Yang
"""

import numpy as np
import cv2

class ConvexHull():
    def __init__(self):
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
    #gets convex hull and bounding rectangle for the rotating object
    def boundingRect(self,frame):
        fgmask = self.fgbg.apply(frame)
        ret, fgmask = cv2.threshold(fgmask, 100,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        fgmask = cv2.dilate(fgmask, None, 25)
        fgmask = cv2.erode(fgmask, None, 25)
        fgmask = cv2.dilate(fgmask, None, 5)

        im2, contours, hierarchy = cv2.findContours(fgmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = sorted(contours,key=cv2.contourArea,reverse=True)[0]
        hull = cv2.convexHull(contour)
        rect = cv2.boundingRect(hull)
        
        return (hull,rect)
