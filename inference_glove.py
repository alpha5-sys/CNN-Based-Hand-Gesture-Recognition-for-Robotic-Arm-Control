#----------------- DEPENDENCIES ------------------#
import serial
from time import sleep, time
import pandas as pd
import numpy as np
import os
#------------------------------------------------#

#----------------- FUNCTIONS ------------------#

def port_auto_detect():
    import warnings
    import serial
    import serial.tools.list_ports
    ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'CH340' in p.description  # may need tweaking to match arduino
    ]
    if not ports:
        raise IOError("No Device found!")
    if len(ports) > 1:
        warnings.warn('Multiple Device found - using the first')
    return ports[0]

#-----------------------------------------------#

#------------------ SETTINGS -------------------#
columnName = ['timestamp', 'user_id',
              'flex_1', 'flex_2', 'flex_3', 'flex_4', 'flex_5',
              'Qw', 'Qx', 'Qy', 'Qz',
              'GYRx', 'GYRy', 'GYRz',
              'ACCx', 'ACCy', 'ACCz',
              'ACCx_body', 'ACCy_body', 'ACCz_body',
              'ACCx_world', 'ACCy_world', 'ACCz_world']

all_data = []  # Store data from all repetitions here
user_id = '010'  # User ID can be changed as needed
gestureName = input("Enter Gesture Name: ").lower()
segmentLength = 150
repetitions = 10  # Number of times to repeat gesture recording

# Permanently set the directory where the CSV file will be saved
save_directory = r'C:\Users\Niklesh\Documents\Capstone Project\Project code\my_dataset_robotic_arm'  # Replace this with your permanent path

# Create the directory if it doesn't exist
store_path = os.path.join(save_directory, user_id)
if not os.path.isdir(store_path):
    os.makedirs(store_path)

port = port_auto_detect()
ser = serial.Serial(port=port,
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1,
                    xonxoff=0,
                    rtscts=0)

print("\nEstablishing Communication... Please Wait")

ser.dtr = False
sleep(1)
ser.reset_input_buffer()
ser.dtr = True
sleep(1)

#-------------------------------------------------#

#------------------ ITERATION SETTINGS -------------------#
count = 0
recordData = True
interruptToken = False
current_time = time()
prev_time = current_time
T = np.zeros(segmentLength, dtype=float)

#-------------------------------------------------#

#------------------ INITIALIZATION -------------------#
print("\nInitializing MPU6050 in ..", end='')

# Wait for valid data to start
while True:
    try:
        ser.readline().decode('utf-8').rstrip().split(',')
        break
    except(ValueError):
        print('.')

# Discard some initial values for stabilization
val_discard = 500
for i in range(val_discard):
    values = ser.readline().decode('utf-8').rstrip().split(',')
    if (i % 100) == 0:
        print(int((val_discard-i) / 100))
print('\n')

#-------------------------------------------------#

#------------------ MAIN ITERATIONS -------------------#

repetition_count = 0

while repetition_count < repetitions:
    try:
        current_time = time()
        values = ser.readline().decode('utf-8').rstrip().split(',')
        values = list(map(float, values))  # Attempt to convert all values to float
        
        # Insert timestamp and user ID
        values.insert(0, current_time)
        values.insert(1, user_id)
        
        print(values)

        if recordData:
            interruptToken = False
            all_data.append(values)
            print(f"Data Received Successfully... index: {len(all_data)-1}")
            print(f"Data values: {values}")  # Print data values to the terminal
            T[(len(all_data)-1) % segmentLength] = current_time

        # Process the segment after every 150 samples
        if len(all_data) % segmentLength == 0 and recordData and not interruptToken:
            count += 1
            print(f'\tSegment - {count} finished')

            # Optional: Check and print the sampling frequency if needed
            if len(T) > 1:
                f_s = 1 / np.diff(T)
                f_s = np.delete(f_s, 0)

            # Mark repetition complete
            repetition_count += 1
            print(f"Repetition {repetition_count}/{repetitions} completed.")
            
            # Reset for next repetition
            T = np.zeros(segmentLength, dtype=float)

    except KeyboardInterrupt:
        # Handle case where the user stops manually
        if not recordData:
            print("\n\nStart")
            ser.readline().decode('utf-8').rstrip().split(',')
            recordData = True

    except ValueError:
        print("Data Recording Failed: ValueError - Possibly malformed data.")

    except Exception as e:
        print(f"Data Recording Failed: {e}")

# After all repetitions, save the entire dataset
df = pd.DataFrame(all_data, columns=columnName)
df.to_csv(os.path.join(store_path, f'{gestureName}.csv'), index=False)
print(f"All data saved as {gestureName}.csv.")
