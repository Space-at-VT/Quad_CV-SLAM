import numpy as np
import cv2
import cvtools

class KLTtracker():
    def __init__(self, frame, kpInit, desInit, roiVec):
        self.lkParams = dict(winSize=(15,15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,10,0.03))
        self.oldFrame = cvtools.GrayBlur(frame)
        self.kp0 = np.array(kpInit)
        self.des0 = desInit
        self.p0 = np.zeros([len(self.kp0),2],dtype = np.float32)
        self.x = roiVec[0]
        self.y = roiVec[1]
        self.w = roiVec[2]
        self.h = roiVec[3]

        self.actPtLife = np.zeros([self.kp0.size,1],dtype = np.int16)
        
        for ii in range(0,self.kp0.size):
            self.p0[ii,:] = (self.kp0[ii].pt[0]+self.x,self.kp0[ii].pt[1]+self.y)
    
    def increment_life(self,st):
        # do this to track lifespan of points. Longer life = trusted track.
        for i in range(len(st)):
            if(st[i,0]==1):
                self.actPtLife[i] += 1
            else:
                self.actPtLife[i] = 0
    def track(self, frame): #tracks similar points between 2 frames
        if(len(self.p0)==0):
            return
        frame_gray = cvtools.GrayBlur(frame)
        
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.oldFrame, frame_gray, self.p0, None, **self.lkParams)
    
        p0 = self.p0.copy()

        #gets points with status of 1
        temp = []
        for i in range(len(st)):
            if(st[i,0]==1):
                temp.append(p0[i])
        p0 = np.array(temp)

        #gets points with status of 1
        temp = []
        for i in range(len(st)):
            if(st[i,0]==1):
                temp.append(p1[i])
        p1 = np.array(temp)
        
        self.increment_life(st)

        #gets points with status of 1
        temp = []
        for i in range(len(st)):
            if(st[i,0]==1):
                temp.append(self.kp0[i])
        self.kp0 = np.array(temp)

        temp = []
        for i in range(len(st)):
            if(st[i,0]==1):
                temp.append(self.des0[i])
        self.des0 = np.array(temp)
        
        self.p0 = np.array(p1.copy())

        self.oldFrame = frame_gray

        return p0, p1
    def match(self, frame, orb, bf): #not complete
        if(len(self.p0)==0):
            return
        frame_gray = cvtools.GrayBlur(frame)
        
        # detect kp, compute des within ROI in new hull image
        img = frame_gray[self.y:self.y+self.h,self.x:self.x+self.w]
        kp1, des1 = orb.detectAndCompute(img,None)
        kp1 = np.array(kp1)
        
        # adjust point coordinates from hull frame to image frame
        for ii in range(0,kp1.size):
            kp1[ii].pt = (kp1[ii].pt[0]+self.x,kp1[ii].pt[1]+self.y)
            
        self.p0 = np.zeros([kp1.size,2],dtype = np.float32)
        for ii in range(0,kp1.size):
            self.p0[ii,:] = (kp1[ii].pt[0],kp1[ii].pt[1])
        
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.oldFrame, frame_gray, self.p0, None, **self.lkParams)
        
        stIdx = np.zeros([kp1.size,1],dtype = np.int16)
         # identify strongest matches between library des and current des
        matches = bf.match(des1,self.des0)
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
        nums = np.linspace(0,len(kp1)-1,len(kp1),dtype=np.int32)
        newPtIdx = np.setdiff1d(nums,mQueryIdx)
        # update library descriptor with matched descriptor
        self.des0[mTrainIdx] = des1[mQueryIdx]
        
        p0 = self.p0.copy()

        temp = []
        for i in range(len(st)):
            if(st[i,0]==1):
                temp.append(p0[i])
        p0 = temp
        
        temp = []
        for i in range(len(st)):
            if(st[i,0]==1):
                temp.append(p1[i])
        p1 = temp
        
        self.increment_life(st)

        temp = []
        for i in range(len(st)):
            if(st[i,0]==1):
                temp.append(kp1[i])
        self.kp0 = temp

        temp = []
        for i in range(len(st)):
            if(st[i,0]==1):
                temp.append(des1[i])
        self.des0 = temp
        
        self.p0 = np.array(p1.copy())

        self.oldFrame = frame_gray

        return p0, p1

