import cv2
import numpy as np
from PIL import Image

# load some images
im1 = np.array(Image.open('images/001.jpg'))
im2 = np.array(Image.open('images/002.jpg'))

# load 2D points for each view to a list
points2D = [loadtxt('2D/00'+str(i+1)+'.corners').T for i in range(3)]

# load 3D points
points3D = loadtxt('3D/p3d').T

# load correspondences
corr = genfromtxt('2D/nview-corners',dtype='int',missing='*')

# load cameras to a list of Camera objects
P = [cv2.Camera(loadtxt('2D/00'+str(i+1)+'.P')) for i in range(3)]