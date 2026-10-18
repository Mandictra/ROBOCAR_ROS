import serial
import time
import math
from vpython import canvas, box, cylinder, vector, color, rate, compound, button, wtext

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'  # Change to your port
BAUD_RATE = 115200

# --- 3D SCENE SETUP ---
scene = canvas(title="IMU Calibration Interface", width=800, height=500, background=color.black)
scene.range = 6

# 3D Robot Model Construction
chassis = box(pos=vector(0, 0.5, 0), length=4, width=2.5, height=0.5, color=color.blue)
payload = box(pos=vector(-0.5, 1.25, 0), length=2.5, width=2.5, height=1, color=color.gray(0.5))
nose_indicator = cylinder(pos=vector(2, 0.5, 0), axis=vector(0.8, 0, 0), radius=0.15, color=color.red)

w_fl = cylinder(pos=vector(1.2, 0.6, 1.25), axis=vector(0, 0, 0.4), radius=0.6, color=color.white)
w_fr = cylinder(pos=vector(1.2, 0.6, -1.65), axis=vector(0, 0, 0.4), radius=0.6, color=color.white)
w_rl = cylinder(pos=vector(-1.2, 0.6, 1.25), axis=vector(0, 0, 0.4), radius=0.6, color=color.white)
w_rr = cylinder(pos=vector(-1.2, 0.6, -1.65), axis=vector(0, 0, 0.4), radius=0.6, color=color.white)

robot = compound([chassis, payload, nose_indicator, w_fl, w_fr, w_rl, w_rr])

# --- UI SETUP ---
scene.caption = "\n"  # Add space below the canvas

# Function triggered when the user clicks the UI button
def send_ready(b):
    if 'ser' in globals() and ser.is_open:
        # Send the newline character that the C++ waitEnter() function is expecting
        ser.write(b'\n')
        status_text.text = "<br><b><span style='color:green;'>Command Sent! Processing...</span></b><br><br>"

# Create the UI Button
btn = button(bind=send_ready, text="<b>I'M READY / NEXT STEP</b>", color=color.black, background=color.cyan)

# Create text areas for status and the C++ terminal output
status_text = wtext(text="<br><b>Status:</b> Waiting for instructions from robot...<br><br>")
terminal_text = wtext(text="<b><span style='color:gray;'>Robot Console Output:</span></b><br>")

# --- STATE VARIABLES ---
roll, pitch, yaw = 0.0, 0.0, 0.0
last_time = 0
console_log = []

# --- SERIAL CONNECTION ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    status_text.text = f"<br><b><span style='color:red;'>Failed to connect to {SERIAL_PORT}: {e}</span></b><br><br>"
    exit()

# --- MAIN LOOP ---
while True:
    rate(50)  
    
    try:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # If it's telemetry, update the 3D model
            if line.startswith("RAW"):
                parts = line.split(',')
                if len(parts) < 9: continue
                
                current_time = int(parts[2])
                
                # Axes matching standard IMU setup
                ax, ay, az = float(parts[3]), float(parts[4]), float(parts[5])
                gx, gy, gz = float(parts[6]), float(parts[7]), float(parts[8])
                
                dt = 0.02 if last_time == 0 else (current_time - last_time) / 1000.0
                last_time = current_time
                
                # Sensor Fusion
                accel_pitch = math.atan2(ay, math.sqrt(ax**2 + az**2))
                accel_roll = math.atan2(-ax, az) 
                
                gyro_pitch = pitch + (gx * dt)
                gyro_roll = roll + (gy * dt)
                yaw = yaw + (gz * dt)
                
                pitch = 0.98 * gyro_pitch + 0.02 * accel_pitch
                roll = 0.98 * gyro_roll + 0.02 * accel_roll
                
                # Update 3D model vectors
                k = vector(math.cos(yaw)*math.cos(pitch), math.sin(pitch), math.sin(yaw)*math.cos(pitch))
                y = vector(-math.cos(yaw)*math.sin(pitch)*math.sin(roll) - math.sin(yaw)*math.cos(roll),
                            math.cos(pitch)*math.sin(roll),
                           -math.sin(yaw)*math.sin(pitch)*math.sin(roll) + math.cos(yaw)*math.cos(roll))
                
                robot.axis = k
                robot.up = y

            # If it's NOT telemetry, it's an instruction or result from the C++ code
            elif len(line) > 0:
                # Highlight the "press ENTER" prompt from the C++ code
                if "press ENTER when ready" in line:
                    status_text.text = "<br><b><span style='color:orange;'>ACTION REQUIRED: Read instructions below, then click 'I'M READY'</span></b><br><br>"
                    line = f"<b><span style='color:orange;'>{line}</span></b>"
                # Highlight RESULT lines
                elif line.startswith("RESULT"):
                    line = f"<span style='color:cyan;'>{line}</span>"
                
                # Add line to our UI log (keep last 12 lines visible)
                console_log.append(line)
                if len(console_log) > 12:
                    console_log.pop(0)
                
                # Update the UI text panel
                formatted_log = "<br>".join(console_log)
                terminal_text.text = f"<b><span style='color:gray;'>Robot Console Output:</span></b><br><div style='font-family: monospace;'>{formatted_log}</div>"

    except KeyboardInterrupt:
        break
    except Exception as e:
        pass