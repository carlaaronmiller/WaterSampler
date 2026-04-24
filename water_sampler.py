from pymavlink import mavutil
from datetime import datetime
from SPFromCPython import SP_from_C
import time
import serial
import glob
import sys

# -------------------------------MAV CMD PARAMS-------------------------------
RC_SWITCH_PORT = 7
PWM_LOW = 1000
PWM_HIGH = 1900
SET_SERVO_CMD_ID = 183
TARGET_SYSTEM = 1
TARGET_COMPONENT = 1
CONFIRMATION = 0
UNUSED_PARAM = 0
MAVLINK_PAUSE_TIME_SECONDS = 1
# -------------------------------MAIN FUNCTION PARAMS-------------------------------
REFRESH_PERIOD_SECONDS = 2
NO_DATA_VAL = -1
SENSOR_ERROR_VAL = -2
TEXT_BACKUP_HEADER = "Time, BAR30-Depth (m), BAR30-Temp (°C), AML Cond (mS/cm), AML Temp (°C), PSU (Calulated), AML Chloro (μg/L), AML Rho (ppb), AML Turb (NTU),  AML DO (μmol/L)\n"
# -------------------------------PHYSICAL CONSTANTS-------------------------------
STANDARD_ATMOSPHERIC_PRESSURE_HPA = 1013.25
DEG_C_PER_DEG_CENTI_C = 0.01
GRAV_ACC = 9.8
SALTWATER_DENSITY_KGM3 = 1023.6
SEC_TO_MICROSEC = 1e6
HPA_TO_PA = 100
HPA_TO_BAR = 100
# -------------------------------CONNECTION SETUP-------------------------------
MASTER_CONN = mavutil.mavlink_connection("tcp:127.0.0.1:5777")
BOOT_TIME = time.time()
ROV_COCKPIT_ADDRESS = "udpout:192.168.2.2:14570"
# -------------------------------SENSOR SETUP-------------------------------
SENSOR_DICT = {"CT.X2": None, "Chloro-blue": None, "Rhodamine": None, "Turbidity": None, "Dissolved Oxygen": None}
SENSOR_BAUD_RATE = 9600
SENSOR_REBOOT_TIME_SECONDS = 10  # Healthy amount of time from power to actually streaming data.
SENSOR_CMD_WAIT_TIME_SECONDS = 1  # Give sensors time to respond to the cmd.
SENSOR_RESPONSE_TIMEOUT_SECONDS = 5  # If nothing after this amount of time, nothings coming.


# -------------------------------FUNCTIONS-------------------------------
def power_cycle_sensors():  # Power cycles the sensors on defined port to trip RC Switch to high.
    print("Powering off sensors.")
    MASTER_CONN.mav.command_long_send(
        TARGET_SYSTEM,
        TARGET_COMPONENT,
        SET_SERVO_CMD_ID,
        CONFIRMATION,
        RC_SWITCH_PORT,
        PWM_LOW,
        UNUSED_PARAM,
        UNUSED_PARAM,
        UNUSED_PARAM,
        UNUSED_PARAM,
        UNUSED_PARAM,
    )
    time.sleep(SENSOR_REBOOT_TIME_SECONDS)
    print("Powering on sensors.")
    MASTER_CONN.mav.command_long_send(
        TARGET_SYSTEM,
        TARGET_COMPONENT,
        SET_SERVO_CMD_ID,
        CONFIRMATION,
        RC_SWITCH_PORT,
        PWM_HIGH,
        UNUSED_PARAM,
        UNUSED_PARAM,
        UNUSED_PARAM,
        UNUSED_PARAM,
        UNUSED_PARAM,
    )
    time.sleep(SENSOR_REBOOT_TIME_SECONDS)


def send_cockpit_value(dest, name, sensor_value):
    dest.mav.named_value_float_send(int((time.time() - BOOT_TIME) * SEC_TO_MICROSEC), name.encode(), sensor_value)


def get_message(mavlink_conn, msg_type):
    msg = None
    while temp := mavlink_conn.recv_match(type=msg_type):
        msg = temp
    if msg is None:
        msg = mavlink_conn.recv_match(type=msg_type, blocking=True)
    return msg


def get_sensor_line(ser_num):
    if not ser_num or not ser_num.is_open:
        print("Attempted to read from closed or invalid serial port.")
        return ""
    try:
        ser_num.flushInput()
        ser_num.flushOutput()
        ser_num.readline()  # discard partial/incomplete line
        return ser_num.readline().decode("utf-8", errors="ignore").strip()
    except serial.SerialException as e:
        print(f"Serial error during read: {e}")
        return ""


def get_ct_nums(sen):
    sensor_line = get_sensor_line(SENSOR_DICT[sen])
    split_line = sensor_line.split()
    ct_vals = [None] * 2
    if len(split_line) < 2:
        print(f"Unexpected sensor output for {sen}: '{sensor_line}'. Expected at least 2 values.")
        return [SENSOR_ERROR_VAL, SENSOR_ERROR_VAL]
    ct_vals[0] = float(split_line[0])  # C
    ct_vals[1] = round((float(split_line[1])), 2)  # T
    return ct_vals


def get_single_val(sen):
    sensor_line = get_sensor_line(SENSOR_DICT[sen])
    try:
        sensor_val = int(float(sensor_line))
    except ValueError:
        sensor_val = SENSOR_ERROR_VAL
    return sensor_val


def discover_devices():
    power_cycle_sensors()  # Trigger the RC switch to give the sensors power.
    start_time = time.time()

    while time.time() - start_time < SENSOR_RESPONSE_TIMEOUT_SECONDS:
        usb_devices = glob.glob("/dev/ttyUSB*")
        if usb_devices:
            break
        time.sleep(SENSOR_CMD_WAIT_TIME_SECONDS)

    if not usb_devices:
        print(f"No USB devices detected after {SENSOR_RESPONSE_TIMEOUT_SECONDS} seconds. Exiting.")
        sys.exit(1)

    for dev in usb_devices:
        try:
            print(f"Probing {dev}...")
            ser = serial.Serial(dev, SENSOR_BAUD_RATE, timeout=SENSOR_CMD_WAIT_TIME_SECONDS)
            time.sleep(SENSOR_CMD_WAIT_TIME_SECONDS)
            ser.write(b"\r")
            time.sleep(SENSOR_CMD_WAIT_TIME_SECONDS)
            ser.write(b"display options\r")

            start_time = time.time()
            found = False
            while time.time() - start_time < SENSOR_RESPONSE_TIMEOUT_SECONDS:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                for s_name in SENSOR_DICT:
                    if SENSOR_DICT[s_name] is None and s_name in line:
                        print(f"{s_name} found on <{dev}>.")
                        SENSOR_DICT[s_name] = ser
                        found = True

            if not found:
                print(f"No matching sensor on {dev}. Closed.")
                ser.close()

        except serial.SerialException as e:
            print(f"Failed to connect to {dev}: {e}")

    return SENSOR_DICT


def setup_cockpit_conn(address):
    rov_cockpit_conn = mavutil.mavlink_connection(address, source_system=1, source_component=1)
    rov_cockpit_conn.mav.ping_send(
        int((time.time() - BOOT_TIME) * SEC_TO_MICROSEC), UNUSED_PARAM, UNUSED_PARAM, UNUSED_PARAM
    )
    time.sleep(MAVLINK_PAUSE_TIME_SECONDS)
    rov_cockpit_conn.recv_match()
    return rov_cockpit_conn


def setup_text_backup():
    file_name = f"/usr/blueos/userdata/sensorData/{datetime.now().date()}.txt"
    backup_file = open(file_name, "a")
    backup_file.write(TEXT_BACKUP_HEADER)
    return backup_file


def write_to_backup(file, line):
    try:
        file.write(line + "\n")
    except Exception as e:
        print(f"Error writing to backup file: {e}")


# ------------------------- MAIN LOOP -----------------------------
if __name__ == "__main__":
    sen_dict = discover_devices()
    rov_cockpit_conn = setup_cockpit_conn(ROV_COCKPIT_ADDRESS)
    text_backup = setup_text_backup()
    text_line = ""

    try:
        while True:
            text_line = datetime.now().strftime("%H:%M:%S")
            print(text_line)
            BAR30_val = get_message(MASTER_CONN, "SCALED_PRESSURE2")
            BAR30_depth = (
                (BAR30_val.press_abs - STANDARD_ATMOSPHERIC_PRESSURE_HPA)
                * HPA_TO_PA
                / (SALTWATER_DENSITY_KGM3 * GRAV_ACC)
            )  # Converting from raw pressure (hPa) to meters.
            BAR30_temp = (BAR30_val.temperature) * DEG_C_PER_DEG_CENTI_C  # Convert from centi°C to  °C.
            text_line += f",{BAR30_depth:.2f},{BAR30_temp:.2f}"

            for sen in sen_dict:
                if sen_dict[sen] is None:
                    send_cockpit_value(rov_cockpit_conn, "AML" + sen, NO_DATA_VAL)
                    text_line += f", {NO_DATA_VAL}"

                elif sen == "CT.X2":  # CT value handling  + Salinity Calc.
                    s_val = get_ct_nums(
                        sen
                    )  # Calculate Salinity (PSU) from Conductivity (mS/cm), Temp (deg C), P (bar).
                    sal_psu = (
                        NO_DATA_VAL
                        if SENSOR_ERROR_VAL in s_val
                        else SP_from_C([s_val[0]], [s_val[1]], [BAR30_depth * HPA_TO_BAR])
                    )
                    print(f"CT: {s_val}, Sal(PSU): {sal_psu}")
                    text_line += f",{s_val[0]},{s_val[1]},{sal_psu}"
                    send_cockpit_value(rov_cockpit_conn, "AML-Cond", s_val[0])
                    send_cockpit_value(rov_cockpit_conn, "AML-Temp", s_val[1])
                    send_cockpit_value(rov_cockpit_conn, "Sal-Calc", sal_psu)

                else:  # Case single value sensor.
                    s_val = get_single_val(sen)
                    if s_val == NO_DATA_VAL or s_val == SENSOR_ERROR_VAL:
                        print(f"Error found in sensor line {sen}. Removing sensor.")
                        SENSOR_DICT[sen] = None
                    else:
                        text_line += f",{s_val}"
                        print(f"{sen}: {s_val}")
                        send_cockpit_value(rov_cockpit_conn, "AML" + sen, s_val)

            print("\n")
            write_to_backup(text_backup, text_line)
            time.sleep(REFRESH_PERIOD_SECONDS)

    except KeyboardInterrupt:
        print("Exiting script...")

    finally:
        text_backup.close()
        for s in SENSOR_DICT.values():
            if isinstance(s, serial.Serial) and s.is_open:
                s.close()
