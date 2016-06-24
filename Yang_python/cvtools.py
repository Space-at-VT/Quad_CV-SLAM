"""
Created on Thu Mar 24 18:06:54 2016

@author: Loren
"""

# global kp0, des0, p0, lk_params

# Modules
import cv2
import numpy as np

def GrayBlur(frame,k1=5,k2=5,sig=0):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame,(k1,k2),sig)
    
    return frame

def getProjectionMatrices(U,S,V):
    W = np.matrix([[0,-1,0],[1,0,0],[0,0,1]])
    PXcam = np.zeros((3,4,4))
    
    #PXcam[:,:,0] = [U*W*V.transpose(),U[:,2]];
    #PXcam[:,:,1] = [U*W*V.transpose(),-U[:,2]];
    #PXcam[:,:,2] = [U*W.transpose()*V.transpose(),U[:,2]];
    #PXcam[:,:,3] = [U*W.transpose()*V.transpose(),-U[:,2]];

    PXcam[0:3,0:3,0] = U*W*V.transpose()
    PXcam[0:3,0:3,1] = U*W*V.transpose()
    PXcam[0:3,0:3,2] = U*W.transpose()*V.transpose()
    PXcam[0:3,0:3,3] = U*W.transpose()*V.transpose()
    PXcam[0:3,3,0] = U[:,2].transpose()
    PXcam[0:3,3,1] = -U[:,2].transpose()
    PXcam[0:3,3,2] = U[:,2].transpose()
    PXcam[0:3,3,3] = -U[:,2].transpose()

    if(np.linalg.det(PXcam[0:3,0:3,0])<0):
        PXcam[0:3,0:3,0] = -PXcam[0:3,0:3,0]
    if(np.linalg.det(PXcam[0:3,0:3,1])<0):
        PXcam[0:3,0:3,1] = -PXcam[0:3,0:3,1]
    if(np.linalg.det(PXcam[0:3,0:3,1])<0):
        PXcam[0:3,0:3,2] = -PXcam[0:3,0:3,2]
    if(np.linalg.det(PXcam[0:3,0:3,1])<0):
        PXcam[0:3,0:3,3] = -PXcam[0:3,0:3,3]

    return PXcam

def getCorrectProjectionMatrix(PXcam, K, p0, p1):
    Pcam = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
    P = K*Pcam

#filters out track noise, not complete yet
def filterTracks(p0, p1):
    dist = []
    for i in range(len(p0)):
        dist += (((p1[i][0]-p0[i][0])**2+(p1[i][1]-p0[i][1])**2), i)

    dist = sorted(dist, key=lambda d: dist[0])

    cut = -1
    largestDist = -1
    for i in range(1,len(dist)):
        distance = dist[i] - dist[i-1]
        if(largestDist==-1):
            largestDist = distance
            cut = i
        elif(distance>largestDist):
            largestDist = distance
            cut = i
            
    newp0 = []
    newp1 = []

    for i in range(cut):
        newp0.append(p0[dist[1]])
        newp1.append(p1[dist[1]])

    return newp0, newp1

    
            
