import serial
import threading
from datetime import datetime

from pynput import keyboard

# Initialisation
COM_PORT = 'COM15'
BAUD_RATE = 115200
LOG_FILE = "motor_pid_data.csv"

motor_started = False
motor_initialised = False
# Checks serial is open for Python to use
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    print(f"[+] pySerial connected to ESP32 on {COM_PORT}")
except Exception as e:
    print(f"[!] pySerial connection failed: {e}. Close other Serial Monitors first!")
    exit()

# Sets up a file to log the data onto
file = open(LOG_FILE, "w")
file.write("Timestamp,P1,P3,P_DROP,SETPT,POS,RPM\n")
print(f"[+] Logging initialized. Saving to {LOG_FILE}")

flush_counter = 0
# If the motor is started
def read_data():
    global motor_started, flush_counter, motor_initialised
    while True:
        try:
            if ser.in_waiting > 0:
                # if more than 0 bytes are received, serial is decoded from UTF-8
                # which the ESP-32 uses
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                # Once decoded into characters, if 'STOP' is received along serial

                if line == "ESTOP":
                    print("\n[!] ESP32 confirms motor stop.")
                    motor_started = False
                    break

                elif line == "SERCON":
                    print("Serial Connection Error to Arduino")
                    break

                elif line == "ADSOFF":
                    print("ADS Failure")

                elif line == "ADSON":
                    print("ADS Restored")

                elif line == "DIVZEROENC":
                    print("Encoder Timing Error")

                elif line == "INITNOW":
                    print("Motor must be initialised - please press i")

                elif line == "INITING":
                    print("Motor beginning initialisation")

                elif line.startswith("INITED"):
                    try:
                        parts = line.split(",")  # Splits "INITED,356.20" into ["INITED", "356.20"]
                        angle_range = float(parts[1])
                        print("Motor initialised, angle range is:", angle_range)
                        motor_initialised = True
                    except (IndexError, ValueError):
                        print(f"[!] Corrupted initialization string received: {line}")

                elif line == "MOTSTART":
                    print("Motor started")
                    motor_started = True
                elif line == "NEEDSTART":
                    print("Motor should be started when ready - please press s")

                elif line:
                    # If a message has not been sent, it checks the timestamp, reads data
                    # And then prints to the output

                    # USE DATETIME INSTEAD

                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    data_points = line.split(",")

                    # Code matches the updated 6-column data payload
                    if len(data_points) == 6:

                        bar_a1 = data_points[0]
                        bar_a3 = data_points[1]
                        p13_drop = data_points[2]
                        setpoint = data_points[3]
                        motor_angle = data_points[4]
                        motor_rpm = data_points[5]



                        if motor_started:
                            # Cleaned monitoring display showing BAR metrics exclusively
                            print(f"[{timestamp}] P1: {bar_a1} bar | P3: {bar_a3} bar | PDROP: {p13_drop} | SETPT: {setpoint}  | POS: {motor_angle} deg | VEL: {motor_rpm} RPM")

                            file.write(f"{timestamp},{bar_a1},{bar_a3},{p13_drop},{setpoint},{motor_angle},{motor_rpm}\n")

                            flush_counter += 1
                            if flush_counter % 40 == 0:
                                file.flush()

                    else:
                        if not line.startswith("ESP32"):
                            print(f"[Diagnostic] Noise or data split mismatch: {line}")

        except Exception as e:
            print(f"Error inside data loop: {e}")
            break


def on_press(key):
    global motor_started, motor_initialised
    try:
        if hasattr(key, 'char') and key.char == 's':

            if not motor_started:
                if motor_initialised:

                    ser.write(b's')

                    print("\n[+] 'S' key detected -> Sending START command to motor...")
                else:
                    print("Please press I to initialise the motor first")

        elif hasattr(key, 'char') and key.char == 'i':

            if not motor_initialised:
                ser.write(b'i')

                print("\n[+] 'I' key detected -> Initialising motor...")

        if key == keyboard.Key.space:
            ser.write(b' ')
            print("\n[!] Spacebar pressed! Sending emergency STOP to ESP32...")

    except Exception as e:
        print(f"Error sending data: {e}")


data_thread = threading.Thread(target=read_data, daemon=True)
data_thread.start()

with keyboard.Listener(on_press=on_press) as listener:
    print("\n=======================================================")
    print("[SYSTEM READY] Instructions:")
    print("Press 'i' on your keyboard to INITIALISE the motor")
    print(" -> Press 's' on your keyboard to START the motor.")
    print(" -> Press 'SPACEBAR' on your keyboard to STOP the motor.")
    print("=======================================================\n")
    listener.join()

file.close()