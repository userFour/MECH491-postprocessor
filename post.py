# -*- coding: utf-8 -*-
"""
Created on Fri Oct 15 10:33:10 2021

@author: phill
"""


import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import cm
import math as math
import time as time
import sys as sys
import datetime

now = datetime.datetime.now()

#
# SETUP GOES HERE *TODO*
#


lines = []
with open('APT code.txt') as f:
    lines = f.readlines()
    f.close()

f2 = open( str(now.strftime("%Y.%m.%d.%H.%M.%S")) +' Output G code.txt', 'a')


for i in range(0,len(lines)):
    
    currentContent = lines[i].split(",")
    currentCommand = currentContent[0]
    
    #GOTO/ x, y [,z [,i ,j ,k] [,feed]]
    if(currentCommand[0:4] == "GOTO"):
        currentContent[0] = currentCommand[5:len(currentCommand)]
        gotoPrefixes = ['X', 'Y', 'Z', 'I', 'J', 'K', 'F']
        
        f2.write("N"+str((i+1)*5))
        for i in range(0,len(currentContent[:])):
            f2.write(str(" " + gotoPrefixes[i])+currentContent[i])
        f2.write("\n")



V = [0, 0, 1]


A = math.pi/180 * -27.54332457
B = math.pi/180 * -25.40287102
C = 0


Rx =[[1, 0, 0],
     [0, math.cos(A), -math.sin(A)],
     [0, math.sin(A),  math.cos(A)]]

Ry =[[math.cos(B), 0, math.sin(B)],
     [0, 1, 0],
     [-math.sin(B), 0, math.cos(B)]]

Rz =[[math.cos(C), -math.sin(C), 0],
     [math.sin(C),  math.cos(C), 0],
     [0, 0, 1]]


V1 = np.matmul(Rx, V)
V2 = np.matmul(Ry, V1)
V3 = np.matmul(Rz, V2)
print(V3)

f2.close()
