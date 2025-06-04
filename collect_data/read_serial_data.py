import csv
import serial
from time import sleep

arduino_port = "COM3"  # Replace with your actual port
fileName = "sensor_data.csv"

print("Connected to Arduino port: " + arduino_port)
file = open(fileName, "a")
print("Created file")

samples = 20000
line = 0
sensor_data = []

# Column names for 5 sensors + label
column_names = ["Sensor1", "Sensor2", "Sensor3", "Sensor4", "Sensor5", "Label"]
sensor_data.append(column_names)

# Start serial connection
ser = serial.Serial(arduino_port, 9600)

# Countdown before starting
print("Starting in:")
for i in range(5, 0, -1):
    print(i)
    sleep(1)
print("Recording started...")

boo = True
while line < samples:
    getData = ser.readline().decode().strip()
    readings = getData.split(",")

    if len(readings) != 5:
        continue

    if line < samples / 2:
        label = "normal"
    else:
        label = "กำหมัด"

    readings.append(label)
    sensor_data.append(readings)
    print(readings)

    if line == samples // 2 and boo:
        print("กำหมัด")
        sleep(2)
        boo = False

    line += 1

# Save to CSV
with open(fileName, 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(sensor_data)

print("Data collection complete!")
file.close()
