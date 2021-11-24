# ------------------------------------------------------------------------------
# 5-Axis Postprocessor Python Implementation
#
#
# Authors: Phillip, Markos   Date: 2021-10-15T10:33:10
#
# ------------------------------------------------------------------------------
#
# This script creates Gcode from APT code for a 5 axis machine (B, and C axis)
#
# ------------------------------------------------------------------------------

import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import cm
from math import *
import time as time
import os
import sys as sys
import datetime

now = datetime.datetime.now()

#
# SETUP GOES HERE *TODO*
#
rapidNextLine = 0
currentToolPosition = ['', '', '', '', ''] # X, Y, Z, B, C

def removeRedundantPosiotionValues(data, currentToolPosition):
    #TODO
    
    return (data)



lines = []

print("Directory: ", end="")
print(os.getcwd())
with open('APT_SimpleMillPlane_ZIG.txt') as f:
#with open('APT code.txt') as f:
    lines = f.readlines()
    f.close()

f2 = open( str(now.strftime("%Y.%m.%d.%H.%M.%S")) +' Output G code.txt', 'a')


for i in range(0,len(lines)):
    
    currentContent = lines[i].split(",")
    currentCommand = currentContent[0]
    
    
    
    
    #GOTO/ x, y [,z [,i ,j ,k] [,feed]]
    if(currentCommand[0:4] == "GOTO"):
        currentContent[0] = currentCommand[5:len(currentCommand)] # Removes "GOTO/" prefix
        gotoPrefixes = ['X', 'Y', 'Z', 'B', 'C', '', 'F']
        
        # Calculate B and C axis position
        if (len(currentContent[:]) >= 4): # If we have i j k values that need processing
            v = np.array([currentContent[3], currentContent[4], currentContent[5]], dtype=float)
            theta_B = acos(v[2])
            theta_C = atan2(v[1], v[0])
            currentContent[3] = str(round(degrees(theta_B),4))
            currentContent[4] = str(round(degrees(theta_C),4))
            currentContent[5] = "";
        
        
        # Write line number and G command
        f2.write("N"+str((i+1)*5))
        if (rapidNextLine == 1):
            f2.write(str(" G00"))
            rapidNextLine = 0;
        else:
            f2.write(str(" G01"))
        
        
            
        # loop over each element
        for i in range(0,len(currentContent[:])):
                if(currentContent[i]!= ""): # If there is data in the cell
                    f2.write(str(" "))
                    f2.write(str(gotoPrefixes[i])+str( round(float(currentContent[i])/25.4, 4) ) )
        f2.write("\n")
        

        

    #CIRCLE/
    if(currentCommand[0:6] == "CIRCLE"):
        currentContent[0] = currentCommand[7:len(currentCommand)] # Removes "CIRCLE/" prefix
        circlePrefixes = [' ', ' ']
        
        f2.write("N"+str((i+1)*5))
        f2.write(str(" G02 OR G03 ... TODO\n"))






    #FEDRAT/MMPM, f
    if(currentCommand[0:11] == "FEDRAT/MMPM"):
        currentContent = lines[i].split(",")
        Feed = currentContent[1]
        f2.write(str("F" +       str( round(float(Feed)/25.4, 1)     )   + "\n"   ))
        
        
    #RAPID
    if(currentCommand[0:5] == "RAPID"): # the next line will have a GOTO command that should be G00
        rapidNextLine = 1
        
        
        
f2.close()
