from pymavlink import mavutil
from datetime import datetime
from SPFromCPython import SP_from_C
import time
import serial
import glob
import sys

# -------------------------------CONNECTION SETUP-------------------------------
master = mavutil.mavlink_connection('tcp:127.0.0.1:5777')
boot_time = time.time()
ROVCp = mavutil.mavlink_connection('udpout:192.168.2.2:14570', source_system=1, source_component=1)

# -------------------------------SENSOR SETUP-------------------------------
sDict = {
    "CT.X2": -1,
    "Chloro-blue": -1,
    "Rhodamine": -1,
    "Turbidity": -1,
    "Dissolved Oxygen": -1
}
#Power cycles the sensors on defined port to trip RC Switch to high.
def powerCycleSensors():
    RCSwitchPort = 7
    switchPWMLow = 1000
    switchPWMHigh = 1900
    setServoCmd = 183
    print("Powering off sensors.")
    master.mav.command_long_send(1,1,setServoCmd,0,RCSwitchPort,switchPWMLow,0,0,0,0,0)
    time.sleep(10)
    print("Powering on sensors.")
    master.mav.command_long_send(1,1,setServoCmd,0,RCSwitchPort,switchPWMHigh,0,0,0,0,0)
    time.sleep(10)
def sendCockpitValue(dest, name, sensorValue):
    dest.mav.named_value_float_send(int((time.time() - boot_time) * 1e6), name.encode(), sensorValue)

def getMessage(connection, msgType):
    msg = None
    while temp := connection.recv_match(type=msgType):
        msg = temp
    if msg is None:
        msg = connection.recv_match(type=msgType, blocking=True)
    return msg

def getSensorLine(serNum):
    if not serNum or not serNum.is_open:
        print("Attempted to read from closed or invalid serial port.")
        return ""
    try:
        serNum.flushInput()
        serNum.flushOutput()
        while serNum.read().decode("utf-8", errors='ignore') != '\n':
            pass
        line = []
        while True:
            b = serNum.read().decode("utf-8", errors='ignore')
            line.append(b)
            if b == '\n':
                serNum.reset_input_buffer()
                serNum.reset_output_buffer()
                return ''.join(line)
    except serial.SerialException as e:
        print(f"Serial error during read: {e}")
        return ""

def getCTNums(sen):
    sensorLine = getSensorLine(sDict[sen])
    splitLine = sensorLine.split()
    sensorNums = [None] * 2
    sensorNums[0] = (float(splitLine[0]))#C
    sensorNums[1] = round((float(splitLine[1])),2)#T
    return sensorNums

def getSingleVal(sen):
    sensorLine = getSensorLine(sDict[sen])
    try:
        sensorVal = int(float(sensorLine))
    except ValueError:
        print(f"Error found in sensor line {sen}. Removing sensor.")
        sDict[sen] = -1
        sensorVal = -1
    return sensorVal

# --------------------------- DEVICE DISCOVERY -----------------------------
#Trigger the RC switch to give the sensors power.
powerCycleSensors()
timeout = 10
start_time = time.time()

while time.time() - start_time < timeout:
    usb_devices = glob.glob('/dev/ttyUSB*')
    if usb_devices:
        break
    time.sleep(0.5)

if not usb_devices:
    print("No USB devices detected after 10 seconds. Exiting.")
    sys.exit(1)

for dev in usb_devices:
    try:
        print(f"Probing {dev}...")
        ser = serial.Serial(dev, 9600, timeout=1)
        time.sleep(0.5)
        ser.write(b"\r")
        time.sleep(1)
        ser.write(b"display options\r")

        start_time = time.time()
        found = False
        while time.time() - start_time < 5:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            for sName in sDict:
                if sDict[sName] == -1 and sName in line:
                    print(f"{sName} found on <{dev}>.")
                    try:
                        sDict[sName] = ser
                        found = True
                    except serial.SerialException as e:
                        print(f"Could not reopen {dev} for {sName}: {e}")
                        sDict[sName] = -1

        if not found:
            print(f"No matching sensor on {dev}. Closed.")
            ser.close()
    except serial.SerialException as e:
        print(f"Failed to connect to {dev}: {e}")

# ------------------------- MAVLINK SETUP -----------------------------
ROVCp.mav.ping_send(int((time.time() - boot_time) * 1e6), 0, 0, 0)
time.sleep(1)
ROVCp.recv_match()

fileName = f"/usr/blueos/userdata/sensorData/{datetime.now().date()}.txt"
textBackup = open(fileName, "a")
textBackup.write("Time, BAR30-Depth (m), BAR30-Temp (°C), AML Cond (mS/cm), AML Temp (°C), PSU (Calulated), AML Chloro (μg/L), AML Rho (ppb), AML Turb (NTU),  AML DO (μmol/L)\n")
textLine = ""
# ------------------------- MAIN LOOP -----------------------------
try:
    while True:
        textLine = datetime.now().strftime("%H:%M:%S")
        print(textLine)
        bar30Val = getMessage(master, 'SCALED_PRESSURE2')
        BAR30Depth = (bar30Val.press_abs - 1013)/100.558 #Converting from raw pressure (hPa) to meters.
        BAR30Temp = (bar30Val.temperature)/100 #Convert from centi°C to  °C.
        textLine += f",{BAR30Depth:.2f},{BAR30Temp:.2f}"

        for sen in sDict:
            if sDict[sen] == -1:
                sendCockpitValue(ROVCp, "AML" + sen, -1)
                textLine +=",-1"
                
            elif sen == "CT.X2": #CT value hadling  + Salinity Calc.
                sVal = getCTNums(sen)
                #Calculate Salinity (PSU) from Conductivity (mS/cm), Temp (deg C), P (dbar).
                #Send arguments as array elements.
                hPa2dBar = 100 #Bar30 Pressure in hPa, SP calc in dBar; 1dBar = 100 hPa. 
                salPSU = -1 if -1 in sVal else SP_from_C([sVal[0]],[sVal[1]],[BAR30Depth*hPa2dBar]) 
                print(f"CT: {sVal}, Sal(PSU): {salPSU}")
                textLine += f",{sVal[0]},{sVal[1]},{salPSU}"
                sendCockpitValue(ROVCp, "AMLCond", sVal[0])
                sendCockpitValue(ROVCp, "AMLTemp", sVal[1])
                sendCockpitValue(ROVCp, "Sal-Calc",salPSU)

            

            else: #Case single value sensor.
                sVal = getSingleVal(sen)
                textLine += f",{sVal}"
                print(f"{sen}: {sVal}")
                sendCockpitValue(ROVCp, "AML" + sen, sVal)
        
        print("\n")
        textBackup.write(textLine + "\n")
        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting script...")

finally:
    textBackup.close()
    for s in sDict.values():
        if isinstance(s, serial.Serial) and s.is_open:
            s.close()
