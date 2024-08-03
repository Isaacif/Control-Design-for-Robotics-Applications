from klampt import *
import math
import os   


world = WorldModel()
fileurdf = "robot.urdf"    
res = world.readFile(fileurdf)
