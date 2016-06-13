import numpy as np
import cv2

class PointReconstructor:
    def __init__(self, rotationRadius, originDist):
        self.rotationRadius = rotationRadius
        self.originDist = originDist

        self.watchPoint = (rotationRadius + originDist,0)
    def buildPoints(self, p0, p1):
        pass
