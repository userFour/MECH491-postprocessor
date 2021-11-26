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

# Imports
import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import cm
from math import *
import time
import os
import sys
import datetime

# Define some variables
lines = []
lineNumber = -5 # Make sure first line number is 0
now = datetime.datetime.now()
outFile = str(now.strftime("%Y.%m.%d.%H.%M.%S")) + " Output G code.txt"
outLine = ""
rapidNextLine = 0
currentToolPosition = ['', '', '', '', ''] # X, Y, Z, B, C

# Start the program
os.system('cls' if os.name == 'nt' else 'clear')
print("Start program!")
print("Directory: ", end="")
print(os.getcwd())

# Gcode header block - TODO
#
#

# Remove redundant positions - TODO
def removeRedundantPosiotionValues(data, currentToolPosition):
    #TODO
    
    return (data)

# Read input data from CLDATA file
with open('APT_SimpleMillPlane_ZIG.txt') as f:
    lines = f.readlines()
    f.close()

# Create the output file and open it in append mode
f2 = open(outFile, 'a')

# Start parsing the CLDATA and output Gcode
for i in range(0,len(lines)):
    
    currentContent = lines[i].split(",")
    currentCommand = currentContent[0]
    
    # GOTO/ x, y [,z [,i ,j ,k] [,feed]]
    if(currentCommand[0:4] == "GOTO"):
        currentContent[0] = currentCommand[5:len(currentCommand)] # Removes "GOTO/" prefix
        gotoPrefixes = ['X', 'Y', 'Z', 'B', 'C', '', 'F']

        # Increment line number, build the string
        lineNumber += 5
        outLine = "N" + str(lineNumber)
        if (rapidNextLine == 1):
            outLine += " G00"
            rapidNextLine = 0
        else:
            outLine += " G01"
        
        # Calculate B and C axis position
        if (len(currentContent[:]) >= 4): # If we have i j k values that need processing
            v = np.array([currentContent[3], currentContent[4], currentContent[5]], dtype=float)
            theta_B = acos(v[2])
            theta_C = atan2(v[1], v[0])
            currentContent[3] = str(round(degrees(theta_B),4))
            currentContent[4] = str(round(degrees(theta_C),4))
            currentContent[5] = ""
  
        # Loop over each element
        for i in range(0, len(currentContent[:])):
            if(currentContent[i]!= ""): # If there is data in the cell
                outLine += " "
                outLine += str(gotoPrefixes[i]) + str(round(float(currentContent[i])/1.0, 4))
        
        # Print the output Gcode line
        outLine += "\n"
        # print(outLine, end="")
        f2.write(outLine)
        

        

    # CIRCLE/
    if(currentCommand[0:6] == "CIRCLE"):
        currentContent[0] = currentCommand[7:len(currentCommand)] # Removes "CIRCLE/" prefix
        circlePrefixes = ['X', 'Y', 'Z', ' ', ' ', ' ', 'I', 'J', 'K', 'F']
        
        # Increment line number, build the string
        lineNumber += 5
        outLine = "N" + str(lineNumber) + " G03 "

        for i in range(len(circlePrefixes)):
            if(circlePrefixes[i] != ' '):
                outLine += str(circlePrefixes[i]) + str(round(float(currentContent[i])/1.00, 4)) + ' '    
        
        # Print the output Gcode line
        outLine += "\n"
        # print(outLine, end="")
        f2.write(outLine)


    # FEDRAT/MMPM, f
    if(currentCommand[0:11] == "FEDRAT/MMPM"):
        currentContent = lines[i].split(",")
        Feed = currentContent[1]

        outLine += str(round(float(Feed)/1.0, 1))

        # Increment line number, build the string
        lineNumber += 5
        outLine = "N" + str(lineNumber) + " F "
            
        # Print the output Gcode line
        outLine += "\n"
        # print(outLine, end="")
        f2.write(outLine)


    # RAPID
    if(currentCommand[0:5] == "RAPID"): # the next line will have a GOTO command that should be G00
        rapidNextLine = 1

f2.close()
#os.remove(outFile) # Uncomment this line to supress output file

print("End program!\n")        
