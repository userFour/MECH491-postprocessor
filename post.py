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
conversionFactor = 25.4
printFlag = False
lineNumber = 0 # Make sure first line number is 5
now = datetime.datetime.now()
outFile = str(now.strftime("%Y.%m.%d.%H.%M.%S")) + " Output G code.txt"
outLine = ""
rapidNextLine = 0
currentToolPosition = ['', '', '', '', ''] # X, Y, Z, B, C

# Functions
# -----------------------------------------------------------------------------
# Remove redundant positions - TODO
def removeRedundantPosiotionValues(data, currentToolPosition):
    #TODO
    
    return (data)

# -----------------------------------------------------------------------------
# Increment line numbers and output text to file
def writeToFile(writeData, num):
    
    printString = "N" + str(num)
    for i in range(len(writeData)):
        if(writeData[i] != None):
            printString += " " + str(writeData[i])
    
    f2.write("\n" + printString)

    return ()

# --------------------- START PROGRAM ---------------------
os.system('cls' if os.name == 'nt' else 'clear')
print("Start program!")
print("Directory: ", end="")
print(os.getcwd())

# Read input data from CLDATA file
with open('APT_Zig_Zag_Smoothed.txt') as f:
    lines = f.readlines()
    f.close()

# Create the output file and open it in append mode
f2 = open(outFile, 'a')
print("Filename: " + outFile)

# Gcode header block <-- TODO: Get startup parameters from input CLDATA
f2.write(";START PROGRAM")
f2.write("\nG20 G90 ;Programming in inches, absolute coordinates")
f2.write("\nG28 X Y Z")
f2.write("\nM03 S2037 ;Spindle on (clockwise), 2037 rpm")
f2.write("\nM08 ;Coolant on (flood)")
f2.write("\n;Begin cutting operations")
#

# Start parsing the CLDATA and output Gcode
for i in range(0, len(lines)):
    
    currentContent = lines[i].split(",")
    currentCommand = currentContent[0]

    # Clear the writeData list
    writeData = [None] * 12 # (11 is max number of parmeters (circle cmd))
    
    # GOTO/ x, y [,z [,i ,j ,k] [,feed]]
    if(currentCommand[0:4] == "GOTO"):

        printFlag = True

        currentContent[0] = currentCommand[5:len(currentCommand)] # Removes "GOTO/" prefix
        gotoPrefixes = ['X', 'Y', 'Z', 'B', 'C', ' ', 'F']

        if (rapidNextLine == 1):
            writeData[0] = "G00"
            rapidNextLine = 0
        else:
            writeData[0] = "G01"
        
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
            if(currentContent[i] != ""): # If there is data in the cell
                # Needs to be (i+1) cause index 0 is for G-command
                writeData[i+1] = str(gotoPrefixes[i]) + str(round(float(currentContent[i])/conversionFactor, 4))

    # CIRCLE/
    if(currentCommand[0:6] == "CIRCLE"):

        printFlag = True

        currentContent[0] = currentCommand[7:len(currentCommand)] # Removes "CIRCLE/" prefix
        circlePrefixes = ['X', 'Y', 'Z', ' ', ' ', ' ', 'I', 'J', 'K', ' ', ' ']
        
        # Logic to assign G02/G02 - Needs generalization to function in XZ, YZ planes
        if(float(currentContent[5]) > 0):
            writeData[0] = "G02"
        else:
            writeData[0] = "G03"

        for i in range(len(circlePrefixes)):
            if(circlePrefixes[i] != ' '):
                writeData[i+1] = str(circlePrefixes[i]) + str(round(float(currentContent[i])/conversionFactor, 4)) + ' '   


    # FEDRAT/MMPM, f
    if(currentCommand[0:11] == "FEDRAT/MMPM"):
        currentContent = lines[i].split(",")
        Feed = currentContent[1]

        outLine += str(round(float(Feed)/25.4, 1))

        # Increment line number, build the string
        # lineNumber += 5
        # outLine = "N" + str(lineNumber) + " F " + str(Feed)
            
        # Print the output Gcode line
        # print(outLine, end="")
        # f2.write(outLine)


    # RAPID
    if(currentCommand[0:5] == "RAPID"): # the next line will have a GOTO command that should be G00
        rapidNextLine = 1

    # Send the data to the file write function
    if(printFlag):
        lineNumber += 5
        writeToFile(writeData, lineNumber)
        printFlag = False

# End main loop

f2.close()
#os.remove(outFile) # Uncomment this line to supress output file

print("End program!\n")
