import serial
import time
import datetime
import mysql.connector

# Establish a connection to your MySQL database
db = mysql.connector.connect(
    host="192.168.43.56",  # replace with your host
    user="rafli",  # replace with your username
    password="Hertzman01",  # replace with your password
    database="gps"  # replace with your database name
)

cursor = db.cursor()

arduino = serial.Serial('COM10', 115200)  # replace 'COM#' with your port
prev_timestamp = datetime.datetime.now()

buffers = {}  # The buffer where we'll store the data temporarily
last_write_time = time.time()  # The last time we wrote the data to the database

while True:
    data = arduino.readline().decode('latin-1').strip()
    data_list = data.split(' ')  # split based on whitespace

    if len(data_list) == 3:
        try:
            lat = float(data_list[1])  # get the latitude as a float
            lon = float(data_list[2])  # get the longitude as a float

            # Check if latitude and longitude values are valid
            if -90 <= lat <= 90 and -180 <= lon <= 180:
                buffers['latitude'] = lat
                buffers['longitude'] = lon
            else:
                print("Invalid latitude or longitude value:", data)
                continue

        except (ValueError, IndexError):
            # Handle cases where latitude or longitude value cannot be converted to float or there is an index error
            print("Invalid data format:", data)
            continue

    elif len(data_list) == 1:
        try:
            lon = float(data_list[0])  # get the longitude as a float

            # Check if longitude value is valid
            if 100 <= lon <= 180:
                buffers['longitude'] = lon
            else:
                try:
                    lat = float(data_list[0])  # get the latitude as a float

                    # Check if latitude value is valid
                    if -90 <= lat <= 100:
                        buffers['latitude'] = lat
                    else:
                        print("Invalid latitude value:", data)
                        continue

                except ValueError:
                    print("Invalid latitude and longitude value:", data)
                    continue

        except ValueError:
            # Handle cases where longitude value cannot be converted to float
            print("Invalid longitude value:", data)
            continue

    else:
        print("Invalid data format:", data)
        continue

    # If both latitude and longitude are present in the buffer, insert the data into the database
    if 'latitude' in buffers and 'longitude' in buffers:
        timestamp = str(datetime.datetime.now())
        lat = buffers['latitude']
        lon = buffers['longitude']
        
        query = "INSERT INTO `gps4` (`timestamp`, `latitude`, `longitude`) VALUES (%s, %s, %s)"
        cursor.execute(query, (timestamp, lat, lon))
        db.commit()
        print("Data inserted into MySQL database")

        # Clear the buffer
        buffers.clear()
        last_write_time = time.time()
