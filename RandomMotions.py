
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
class PressureRampWithZero:
    def __init__(self, update_interval=5, seed=None, maxP=30):
        """
        Initializes the ramp generator.
        One component is always 0; the others ramp to random values between 0 and 35.
        """
        self.update_interval = update_interval
        self.last_update_time = 0
        self.maxP = maxP

        if seed is not None:
            np.random.seed(seed)

        self.zero_index = np.random.randint(0, 3)
        self.prev_targets = np.zeros(3)
        self.next_targets = self._generate_targets()

    def _generate_targets(self):
        targets = np.random.uniform(0, self.maxP, 3)
        zero_index = np.random.randint(0, 3)
        targets[zero_index] = 0
        self.zero_index = zero_index
        return targets

    def get_pressures(self, t):
        if t - self.last_update_time >= self.update_interval:
            self.prev_targets = self.next_targets
            self.next_targets = self._generate_targets()
            self.last_update_time = t

        # Progress between 0 and 1
        progress = (t - self.last_update_time) / self.update_interval
        progress = np.clip(progress, 0, 1)

        # Linear interpolation
        current_pressures = self.prev_targets + progress * (self.next_targets - self.prev_targets)

        # Ensure the correct component is zero
        current_pressures[self.zero_index] = 0
        return current_pressures


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


def parse_message(message):
    # Decode the byte message to a string
    decoded_message = message.decode('utf-8').strip()  # Remove any trailing newlines or carriage returns

    # Split the string into individual float strings
    float_strings = decoded_message.split(',')

    # Convert the strings to floats and append to a list
    float_list = [float(num) for num in float_strings]

    return float_list

ser = serial.Serial(port= ComPort, baudrate=baudRate, timeout=1)  # create Serial Object, baud = 9600, read times out after 10s

print("Connected")

startTime = time.time()
prevTime = 0

controller = PressureRampWithZero(update_interval=2, seed=1, maxP=35)


while(True): # create our loop
    msg = ser.readline()  # read serial line til \n

    if len(msg) > 0:

        try:
            parsed_msg = parse_message(msg)  # parse message in form b'float,float,float\r\n'

            t = time.time()-startTime
            dt = t - prevTime


            if dt > 1 / ReadFrequency and not parsed_msg[-1]:  # check if enough time has passed for us to store a message
                prevTime = t
                Pressures = controller.get_pressures(t)
                P1 = Pressures[0]
                P2 = Pressures[1]
                P3 = Pressures[2]
                print(Pressures)

                packAndSendMsg(P1, P2, P3)

                buffer.append([t, P1, P2, P3])  # store message in buffer

                if len(buffer) >= BATCH_SIZE:  # when buffer is filled, write messages to the file and clear buffer
                    write_to_csv_periodic(output_file, buffer)
                    print(f"Batch written. Cycle #{t}")
        except UnicodeDecodeError:
            print("UnicodeDecodeError")

        except ValueError:
            print("Invalid input. Please enter a number.")