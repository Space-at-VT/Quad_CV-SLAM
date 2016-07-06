"""
Created on Thu Mar 24 18:06:54 2016

@author: Loren
"""

# global kp0, des0, p0, lk_params

# Modules
import cv2
import numpy as np
import math

def GrayBlur(frame,k1=5,k2=5,sig=0):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame,(k1,k2),sig)
    
    return frame

def getProjectionMatrices(U,S,V):
    W = np.array([[0,-1,0],[1,0,0],[0,0,1]])
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
    Pcam = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
    P = K*Pcam

    testP0 = p0[0].copy()
    testP0 = np.matrix(np.append(testP0, 1))
    
    testP1 = p1[0].copy()
    testP1 = np.matrix(np.append(testP1, 1))
    
    p0_hat = np.linalg.inv(K)*testP0.transpose()
    X3D = np.zeros((4,4));
    depth = np.zeros((4,2));

    for i in range(3):
        p1_hat = np.linalg.inv(K)*testP1.transpose()
        
        A = np.matrix([np.multiply(Pcam[2,:],p0_hat[0,0])-Pcam[0,:],
                       np.multiply(Pcam[2,:],p0_hat[1,0])-Pcam[1,:],
                       np.multiply(PXcam[2,:,i],p1_hat[0,0])-PXcam[0,:,i],
                       np.multiply(PXcam[2,:,i],p1_hat[1,0])-PXcam[1,:,i]])

        #A1n = math.sqrt(sum(A[0,:].*A[0,:]))
        #A2n = math.sqrt(sum(A[1,:].*A[1,:]))
        #A3n = math.sqrt(sum(A[0,:].*A[0,:]))
        #A4n = math.sqrt(sum(A[0,:].*A[0,:]))

        A1n = np.sqrt(np.inner(A[0,:],A[0,:]))
        A2n = np.sqrt(np.inner(A[1,:],A[1,:]))
        A3n = np.sqrt(np.inner(A[2,:],A[2,:]))
        A4n = np.sqrt(np.inner(A[3,:],A[3,:]))

        Anorm = np.vstack((A[0,:]/A1n,
                A[1,:]/A2n,
                A[2,:]/A3n,
                A[3,:]/A4n))

        Uan,San,Van = np.linalg.svd(Anorm)

        X3D[:,i] = Van[:,-1].reshape(4)

        xi = np.dot(PXcam[:,:,i],X3D[:,i])
        w = xi[2]
        T = X3D[-1,i]
        m3n = np.sqrt(np.inner(Pcam[2,0:3],Pcam[2,0:3]))
        depth[i,1] = (np.sign(np.linalg.det(Pcam[:,0:3]))*w)/(T*m3n)

    if(depth[0,0]>0 and depth[0,1]>0):
        P = PXcam[:,:,0]
    elif(depth[1,0]>0 and depth[1,1]>0):
        P = PXcam[:,:,1]
    elif(depth[2,0]>0 and depth[2,1]>0):
        P = PXcam[:,:,2]
    elif(depth[3,0]>0 and depth[3,1]>0):
        P = PXcam[:,:,3]
    
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

    
            
