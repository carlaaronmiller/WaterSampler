from pymavlink import mavutil
import time
import serial
import re
import subprocess
import glob
import sys


#Connection for depth request
master = mavutil.mavlink_connection('tcp:127.0.0.1:5777')

#Cockpit general setup:
boot_time = time.time()
ROVCp = mavutil.mavlink_connection('udpout:192.168.2.2:14570',source_system=1,source_component=1)

#Function returns the most recent message of the designated msgType. 
def getMessage(connection, msgType):
	msg = None
	while temp := connection.recv_match(type=msgType):
		msg = temp
	if msg == None:
		msg = connection.recv_match(type=msgType, blocking = True)
	return msg

#Sends a message of type "name" to the specific host with the sensor value. 
def sendCockpitValue(dest,name,sensorValue):
	dest.mav.named_value_float_send(int((time.time() - boot_time)*microSecToSec),name.encode(),sensorValue)

#Parsing script for AML Conductivity / Temperature sensor.
def getCTNums(serNum):
	#Get a line from the sensor.
	sensorLine = getSensorLine(serNum)
	#Parse it for the the specific values in said line.
	splitLine = sensorLine.split()
	sensorNums = [None] * 2
	sensorNums[0] = int(float(splitLine[0])*1e5) #Conductivity Value.
	sensorNums[1] = int(float(splitLine[1])) #Temperature Value.
	return sensorNums


#Function to pull a line from the sensor attached from the argument serial number.
def getSensorLine(serNum):
	#Initalize an empty buffer and an empty string.
	line = []
	fullLine = ""
	#Wait for the end of the next line to start reading a line.
	while serNum.read().decode("utf-8",errors='ignore')!='\n':
		pass
	#Read bytes in one at a time, the join and return the full line.
	while True:
		b = (serNum.read()).decode("utf-8",errors='ignore')
		line.append(b)
		if b =='\n':
			fullLine = ''.join(line)
			print("Full line is: ",fullLine)
			line = []
			serNum.reset_input_buffer()
			serNum.reset_output_buffer()
			return fullLine
			

#AML Turbidity implementation
def getTurbVal(serNum):
	#Get a line from the sensor.
	sensorLine = getSensorLine(serNum)
	#Parse it for the the specific values in said line.
 
	return int(float(sensorLine))

#Helper function to check if any data is being transmitted accross the USB line.
def waitForSensorResponse(ser, timeout=5):
    #Wait for data from the sensor for up to timeout seconds.
    ser.timeout = 0.5  # Read timeout (non-blocking)
    start_time = time.time()
    buffer = ''

    while time.time() - start_time < timeout:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                return line
        except Exception:
            pass
    return None  # No valid data received

# SETUP: Figure out which serial port is connected to which AML
serial_connections = []
timeout = 10  # seconds
start_time = time.time()

usb_devices = []

# Wait until USB devices are detected or timeout
while time.time() - start_time < timeout:
    usb_devices = glob.glob('/dev/ttyUSB*')
    if usb_devices:
        break
    time.sleep(0.5)  # Check every 0.5 seconds

if not usb_devices:
    print("No USB devices detected after 10 seconds. Exiting.")
    sys.exit(1)

# Proceed if USB devices found
for dev in usb_devices:
    try:
        ser = serial.Serial(dev, 9600)
        serial_connections.append(ser)
        print(f"Connected to {dev}")

        line = waitForSensorResponse(ser)

        if line is None:
            print(f"No data received from {dev} after 5 seconds. Closing.")
            ser.close()
            continue  # Skip this device

        if line.find(" ") != -1:
            print("It's a CT sensor.")
            CTSerial = ser
        else:
            print("It's not CT.")
            TurbSerial = ser

    except serial.SerialException as e:
        print(f"Failed to connect to {dev}: {e}")


#IMPROTANT: Need to ping the host first for the autopilot to accept the connection.
#Conversion of time unit for timestamping.
microSecToSec = 1e6
ROVCp.mav.ping_send(int((time.time()-boot_time)*microSecToSec),0,0,0)
time.sleep(1)
#Wait for acknowledgement.
msg = ROVCp.recv_match()


#Main function.
#Send the depth reading to CP.
while True:

  CTVals = getCTNums(CTSerial)
  TurbVal = getTurbVal(TurbSerial)
	
  #Print values to the terminal.
  print("Turb: ",TurbVal)
  print("CT: ",CTVals)
  sendCockpitValue(ROVCp,"Cond",CTVals[0])
  sendCockpitValue(ROVCp,"Temp",CTVals[1])
  sendCockpitValue(ROVCp,"Turb",TurbVal)
  depth = getMessage(master,'SCALED_PRESSURE2')
  print(depth.press_abs)
  sendCockpitValue(ROVCp,"BarDepth",float(depth.press_abs))
  time.sleep(1)
