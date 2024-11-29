
import cv2
import numpy as np
import serial
import time
import math
import csv
import matplotlib.pyplot as plt

global ser
global xVals
global yVals
global tik
global tok

tik = time.time() #start clock
tok = tik

################################################################################################
## Define some functions
################################################################################################
def packAndSendMsg(P1, P2, P3):
    #Packs together our message, taking the command character and the text entries and sends it over serial
    global ser
    #print([P1, P2, P3])
    msg = 'A'+ ',' + str(P1) + ',' + str(P2) + ',' + str(P3) # Build Message
    msg = msg + 'Z'  # add end of message indicator
    ser.write(bytes(str(msg), 'UTF-8'))

global error, prevError, errorIntegral, t, xCenter, yCenter
errorIntegral=0
prevError = 0
error = 0
t=0
xCenter = 318
yCenter = 241

def GetContinuumRobotControl():
    global xVals, yVals, tik, tok, errorIntegral, prevError,error,t, xCenter, yCenter

    freq = 0.025
    maxP = 100
    pi = 3.14159

    prevT = t
    tok = time.time()  # get current time
    t = tok - tik  # time elapsed since start of program
    dt = t - prevT

    #for circular path
    #thetaDesired = (t * 2 * pi * freq) % (2 * pi)
    #RadiusDesired = 150


    r = 100

    # calculate actual radius from robot's starting point
    ActualRadius = math.sqrt((xVals[-1]-xCenter)**2 + (yVals[-1]-yCenter)**2)

    cycleT = t%(1/freq)
    if cycleT >=0 and cycleT <=(1/freq)*.25:
        xDes = r
        yDes = -r + cycleT*2*r/(.25/freq)
    elif cycleT >(1/freq)*.25 and cycleT <=(1/freq)*.5:
        xDes = r - (cycleT-.25/freq)*2*r/(.25/freq)
        yDes =r
    elif cycleT > (1 / freq) * .5 and cycleT <= (1 / freq) * .75:
        xDes = -r
        yDes =r - (cycleT-.5/freq)*2*r/(.25/freq)
    else:
        xDes = -r + (cycleT-.75/freq)*2*r/(.25/freq)
        yDes = -r

    RadiusDesired = math.sqrt(xDes ** 2 + yDes ** 2)
    thetaDesired = math.atan2(yDes, xDes)
    prevError = error
    error = RadiusDesired - ActualRadius

    errorIntegral = errorIntegral + dt *(1/2)*(prevError+error) # using trapezoidal integration

    kp = .1
    ki = .5

    if thetaDesired>=0 and thetaDesired<= (2*pi/3):
        P1 = (maxP/2) + math.cos(thetaDesired*6/4)*(maxP/2+ kp*error + errorIntegral*ki)
        P2 = (maxP/2) + math.cos(thetaDesired*6/4-pi)*(maxP/2+ kp*error + errorIntegral*ki)
        P3 = 0
    elif thetaDesired>(2*pi/3) and thetaDesired<= (4*pi/3):
        P1 = 0
        P2 = (maxP/2) + math.cos(thetaDesired*6/4-pi)*(maxP/2+ kp*error + errorIntegral*ki)
        P3 = (maxP/2) + math.cos(thetaDesired*6/4)*(maxP/2+ kp*error + errorIntegral*ki)
    else:
        P1 = (maxP/2) +math.cos(thetaDesired*6/4-pi)*(maxP/2+ kp*error + errorIntegral*ki)
        P2 = 0
        P3 = (maxP/2) + math.cos(thetaDesired*6/4)*(maxP/2+ kp*error + errorIntegral*ki)


    # P1 =  int(maxP / 2. +  math.sin(thetaDesired ) * ((maxP / 2.) + kp*error + errorIntegral*ki))
    # P2 =  int(maxP / 2. +  math.sin(thetaDesired + 120.0 * pi / 180.) * ( (maxP / 2.) + kp*error + errorIntegral*ki))
    # P3 = int(maxP / 2. +  math.sin( thetaDesired + 240.0 * pi / 180.)* ((maxP / 2.) + kp*error + errorIntegral*ki))

    # force P to be bounded
    P1 = int(max(0,min(P1, maxP)))
    P2 = int(max(0, min(P2, maxP)))
    P3 = int(max(0, min(P3, maxP)))


    return P1, P2, P3, RadiusDesired, thetaDesired

ser = serial.Serial(port= 'COM6', baudrate=9600, timeout=10)  # create Serial Object, baud = 9600, read times out after 10s
time.sleep(.1)  # delay 3 seconds to allow serial com to get established
print("Connected")

################################################################################################
## Robot Controls
################################################################################################
#connect camera
video_0 = cv2.VideoCapture(1)

#specify color HSV bounds
# lower boundary RED color range values; Hue (0 - 10)
lower1 = np.array([0, 30, 100])
upper1 = np.array([20, 255, 255])

# upper boundary RED color range values; Hue (160 - 180)
lower2 = np.array([160, 30, 100])
upper2 = np.array([179, 255, 255])

# create empty list which will store our trajectory data
xVals = []
yVals = []
radVals = []
thetaVals = []
P1Vals = []
P2Vals = []
P3Vals = []

while(True): # create our loop
########################################################################################
# set up computer vision
########################################################################################
    #get video frames
    retu, frame = video_0.read()

    #get hsv colors
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #find color masks
    # mask = cv2.inRange(hsv, lower_bound, upper_bound)
    lower_mask = cv2.inRange(hsv, lower1, upper1)
    upper_mask = cv2.inRange(hsv, lower2, upper2)
    mask = lower_mask + upper_mask
    # define kernel size
    kernel = np.ones((7, 7), np.uint8)

    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    color1 = cv2.bitwise_and(frame, frame, mask=mask)


#code to get specific color
    # x = 290
    # y = 290
    # colorsB = frame[y,x,0]
    # colorsG = frame[y,x,1]
    # colorsR = frame[y,x,2]
    # colors = frame[y,x]
    # hsv_value= np.uint8([[[colorsB ,colorsG,colorsR ]]])
    # hsv = cv2.cvtColor(hsv_value,cv2.COLOR_BGR2HSV)
    # print ("HSV : " ,hsv)
    #
    # # Print the HSV value
    #
    # frame[x-1:x+1, y-1:y+1] = [0, 255, 0]

    #finds contours from colors
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #array of center points of contours
    C = np.empty([len(contours), 2], 'i')

    # Draw contour on original image
    output = cv2.drawContours(frame, contours, -1, (0, 0, 255), 2)

    #finds centerpoint of colored dots... adds to array and adds dot to image
    if len(contours) > 0:
        for i in range(len(contours)):
            M = cv2.moments(contours[i])
            C[i, 0] = int(M['m10'] / M['m00'])  # cx
            C[i, 1] = int(M['m01'] / M['m00'])  # cy
            output[C[i, 1] - 2:C[i, 1] + 2, C[i, 0] - 2:C[i, 0] + 2] = [255, 255, 255]
            output = cv2.putText(output, str(i), (C[i, 0], C[i, 1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (245, 244, 66),2, cv2.LINE_AA)

        #also lets add the center point of contour 0 to our list of coordinates
        # xVals.append(C[0, 0].item())
        # yVals.append(C[0, 1].item())
        #print([xVals[-1], yVals[-1]])

        if len(contours) > 1:
            #print("Error: Multiple Centers")
            cv2.line(output, (C[0,0],C[0,1]),(C[1,0],C[1,1]), (255, 0, 0), 2)
        else:

            xVals.append(C[0, 0].item())
            yVals.append(C[0, 1].item())
            print([xVals[-1], yVals[-1]])
            P1, P2, P3, rad, theta= GetContinuumRobotControl()

            radVals.append(rad)
            thetaVals.append(theta)
            P1Vals.append(P1)
            P2Vals.append(P2)
            P3Vals.append(P3)

            packAndSendMsg(P1, P2, P3)


    #Show video with contours
    cv2.imshow('Output', output)
    #cv2.imshow('mask', mask)


    if cv2.waitKey(1) & 0xFF==ord('a'):
        break

video_0.release()
cv2.destroyAllWindows()

output_file = 'robot_coordinates_Square.csv'

# Write to CSV
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Write header
    writer.writerow(['X', 'Y'])
    # Write data
    for x, y in zip(xVals, yVals):
        writer.writerow([x, y])

print(f"Coordinates have been saved to {output_file}")
meanX = sum(xVals) / len(xVals)
meanY = sum(yVals) / len(yVals)
print("Mean X,Y:"+str(meanX) +" "+str(meanY) )

desX = []
desY = []

for i in range(len(thetaVals)):
    desX.append(radVals[i]*math.cos(thetaVals[i]) + xCenter)
    desY.append(radVals[i] * math.sin(thetaVals[i]) + yCenter)


fig, (ax1,ax2) = plt.subplots(1,2)

ax1.plot(xVals, yVals, color='r')

ax1.plot(desX, desY, color='g')

ax2.plot(P1Vals)
ax2.plot(P2Vals)
ax2.plot(P3Vals)

# Display the plot
plt.show()

# plt2.plot(P1, color='r')
#
# # Display the plot
# plt2.show()