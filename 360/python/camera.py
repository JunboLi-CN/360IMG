from math import sin, cos, tan, pi
from vector3 import *

class camera:

    def __init__(self, center=Vector3(0,0,0), up=Vector3(0,0,1), forward=Vector3(0,-1,0), right=Vector3(1,0,0)):
        self.center = center
        self.up = up
        self.forward = forward
        self.right = right

    def setFovAngle(self, fovAngle=90):
        # Calculate the focus
        self.focus = 1.0 / tan((fovAngle * pi / 180.0) / 2.0)

    def setRay(self, x, y):
        self.raypos = self.center
        self.raydir = (self.right * x + self.up * y + self.forward * self.focus)
        self.raydir.normalize()
