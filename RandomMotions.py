
import numpy as np
import serial
import time
import math
import csv
import os


global ser


ComPort = 'COM6'
output_file = 'C:/Users/Ericw/OneDrive/Desktop/PythonDataOutput/SimpleCR.csv'
BATCH_SIZE = 100  # Write to file every 10 entries
ReadFrequency = 50 #hz -- 50 for fatigue, 100 for isotonic, 1000 for response time
baudRate = 115200

# throw error if file already exists
if os.path.exists(output_file):
    raise FileExistsError(f"Error: The file '{output_file}' already exists. Choose a different name.")

buffer = []

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

def write_to_csv_periodic(filename, buffer):
    """Write buffered data to a CSV file and clear buffer."""
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(buffer)
    buffer.clear()  # Clear buffer after writing

## Controller for the CR
def GetContinuumRobotControl(t):


    maxP = 35

    P1 =
    P2 =
    P3 =

    # force P to be bounded
    P1 = int(max(0,min(P1, maxP)))
    P2 = int(max(0, min(P2, maxP)))
    P3 = int(max(0, min(P3, maxP)))

    return P1, P2, P3

ser = serial.Serial(port= ComPort, baudrate=baudRate, timeout=1)  # create Serial Object, baud = 9600, read times out after 10s

print("Connected")

startTime = time.time()
prevTime = 0

while(True): # create our loop

    t = time.time()-startTime
    dt = t - prevTime
    prevTime = t

    if dt > 1 / ReadFrequency:  # check if enough time has passed for us to store a message
        P1, P2, P3 = GetContinuumRobotControl(t)

        packAndSendMsg(P1, P2, P3)

        buffer.append([t, P1, P2, P3])  # store message in buffer

        if len(buffer) >= BATCH_SIZE:  # when buffer is filled, write messages to the file and clear buffer
            write_to_csv_periodic(output_file, buffer)
            print(f"Batch written. Cycle #{t}")