from pymavlink import mavutil
import time
import serial


#connection for depth request
#TODO: check the values for watersampler config.
master = mavutil.mavlink_connection('tcp:127.0.0.1:5777')

#Cockpit
#General setup:
#Config serial connections and cockpit.
#ROV is 192.168.2.1?
boot_time = time.time()
ROVCp = mavutil.mavlink_connection('udpout:192.168.2.1:14570',source_system=1,source_component=1)

#Function returns the most recent message of the designated msgType. 
def getMessage(connection, msgType):
	msg = None
	while temp := connection.recv_match(type=msgType):
		msg = temp
	if msg == None:
		msg = connection.recv_match(type=msgType, blocking = True)
	return msg


#Main function.
#Send the depth reading to CP.
while True:
  #TODO: check which param to request BAR depth.
  depth = getMessage(master,'GPS_RAW_INT')
  sendCockpitValue(ROVCp,"BarDepth",depth)
  time.sleep(1)
