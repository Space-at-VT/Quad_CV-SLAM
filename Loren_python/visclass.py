# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 12:30:03 2016

@author: Loren
"""

import cv2
import numpy as np

class VidFile:
    """
    Class VidFile
    -------------
    file
    title
    fps
    size
    width
    height
    lenDiag
    K
    """
    
    def __init__(self,vidFilePath,vidTitle):
        self.file = cv2.VideoCapture(vidFilePath)
        self.title = vidTitle
        self.fps = self.file.get(cv2.cv.CV_CAP_PROP_FPS)
        self.size = (int(self.file.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)),
                     int(self.file.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)))
        self.width = int(self.file.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
        self.height = int(self.file.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
        self.lenDiag = np.sqrt(np.power(self.width,2) + np.power(self.height,2))
        self.lenDiag = np.int16(self.lenDiag)
        self.K = np.array([[self.width,0,self.width/2],
                           [0,self.width,self.height/2],[0,0,1]])
        
    def close_out(self):
        cv2.destroyAllWindows()
        self.file.release()
        
class PointCloud:
    """
    Class PointCloud
    -------------
    kp0
    des0
    actPtLife
    p0
    x
    y
    """
    
    def __init__(self,kpInit,desInit,roiVec):
        self.kp0 = kpInit
        self.des0 = desInit
        self.actPtLife = np.zeros([self.kp0.size,1],dtype = np.int16)
        # get coordinates from kp to x,y adjusted for ROI
        self.p0 = np.zeros([len(self.kp0),2],dtype = np.float32)
        self.x = roiVec[0]
        self.y = roiVec[1]
        self.w = roiVec[2]
        self.h = roiVec[3]
        for ii in range(0,self.kp0.size):
            self.kp0[ii].pt = (self.kp0[ii].pt[0]+self.x,self.kp0[ii].pt[1]+self.y)
            self.p0[ii,:] = self.kp0[ii].pt
     
    def increment_life(self,idx):
        # do this to track lifespan of points. Longer life = trusted track.
        self.actPtLife[idx[:,0]==1] += 1
        self.actPtLife[idx[:,0]!=1] = 0
       
    def update_lib(self,kp,des,ptIdx):
        #
        newKp = np.array(kp[ptIdx])
        newDes = des[ptIdx]
        newPts = np.zeros([newKp.size,2],dtype = np.float32)
        for ii in range(0,newKp.size):
            newPts[ii,:] = newKp[ii].pt
        self.kp0 = np.append(self.kp0,newKp,axis = 0)
        self.des0 = np.append(self.des0,newDes,axis = 0)
        self.p0 = np.append(self.p0,newPts,axis = 0)
        ptLifeZeros = np.zeros([newKp.size,1],dtype = np.int16)
        
        
