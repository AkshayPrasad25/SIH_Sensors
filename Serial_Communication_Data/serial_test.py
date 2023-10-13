import serial
import csv

# Create a CSV file for writing
csv_file = open('serial_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(line)  # Optional: Print the received data to the console
                # Write the received data to the CSV file
                csv_writer.writerow([line])
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        csv_file.close()
