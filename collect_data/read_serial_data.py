import csv
import serial
from time import sleep

arduino_port = "COM5"  # Replace with your actual port
fileName = "sensor_data.csv"

print("Connected to Arduino port: " + arduino_port)
file = open(fileName, "a")
print("Created file")

samples = 200
line = 0
sensor_data = []

# Column names for 5 sensors + label
# Column names corresponding to the Serial.print output
column_names = [
    "timestamp_ms", "motion_detected",
    "roll_deg", "pitch_deg", "yaw_deg",
    "filtered_accel_x_(m/s²)", "filtered_accel_y_(m/s²)", "filtered_accel_z_(m/s²)",
    "filtered_gyro_x_(rad/s)", "filtered_gyro_y_(rad/s)", "filtered_gyro_z_(rad/s)",
    "accel_magnitude", "gyro_magnitude",
    "adc0", "adc1", "adc2", "adc3", "adc4"
]

sensor_data.append(column_names)

# Start serial connection
ser = serial.Serial(arduino_port, 115200)

# Countdown before starting
print("Starting in:")
for i in range(5, 0, -1):
    print(i)
    sleep(1)
print("Start collecting data...")

boo = True

while line < samples:
    getData = ser.readline().decode().strip()
    readings = getData.split(",")

    # Ensure correct number of readings
    if len(readings) != 18:
        continue  # skip invalid lines

    # Add label and notify user
    if line < samples / 2:
        label = "normal"
        if line == 0:
            print("Label now: normal")
    else:
        label = "กำหมัด"
        if line == samples // 2:
            print("Label now: กำหมัด")
            print("Please กำหมัด (make a fist) now!")
            print("Starting in:")
            for i in range(5, 0, -1):
                print(i)
                sleep(1)
            print("Start collecting data...")

    readings.append(label)
    sensor_data.append(readings)
    print(readings)

    line += 1

# Save to CSV
with open(fileName, 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(sensor_data)

print("Data collection complete!")
file.close()
