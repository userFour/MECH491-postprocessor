# -*- coding: utf-8 -*-
"""
Created on Fri Oct 15 10:33:10 2021

@author: phill
"""


import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import cm
from math import *
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
        currentContent[0] = currentCommand[5:len(currentCommand)] # Removes "GOTO/" prefix
        gotoPrefixes = ['X', 'Y', 'Z', 'A', 'C', '', 'F']
        
        if (len(currentContent[:]) >= 4): # this means that we have i j k values that need processing
            v = np.array([currentContent[3], currentContent[4], currentContent[5]], dtype=float)
            theta_A = acos(v[2])
            theta_C = atan2(v[0], v[1])
            currentContent[3] = str(round(degrees(theta_A),4))
            currentContent[4] = str(round(degrees(theta_C),4))
            currentContent[5] = "";
        
        
        
        f2.write("N"+str((i+1)*5))
        for i in range(0,len(currentContent[:])):
            if(currentContent[i]!= ""):
                f2.write(str(" "))
            f2.write(str(gotoPrefixes[i])+currentContent[i])
        f2.write("\n")

        
f2.close()
