import serial
import time
from typing import List


# Serial port settings
port1 = 'COM3'  # Adjust as necessary
port2 = 'COM4'  # Adjust as necessary


class SerialPort:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def write(self, command):
        self.ser.write((command + '\n').encode())

    def read(self):
        return self.ser.readline().decode().strip()

    def close(self):
        self.ser.close()
    

def send_and_receive(ser1:SerialPort, ser2:SerialPort, commands:List[str], iterations:int):
    total_time = 0
    total_bytes = 0
    st = time.time()
    for i in range(iterations):
        for command in commands:
            start_time = time.time()
            ser1.write(command)


            # Read the response from ser2
            response = ser2.read()
            end_time = time.time()

            if response == command:
                round_trip_time = end_time - start_time
                total_time += round_trip_time
                total_bytes += len(command)

            else:
                print(f"Mismatch: Sent '{command}', Received '{response}'")
    et = time.time()
    ser1.close()
    ser2.close()

    return et-st, total_bytes


def connect(serial_port):
    dta = serial_port.read()
    while dta != '(ok)':
        print(dta)
        dta = serial_port.read()


def main():
    baudrate = 115200

    # G-code commands to test
    gcode_commands = ["G0 X10 Y10", "G1 X20 Y20", "G28", "M114", "M105"]

    # Number of iterations
    iterations = 1000
    ser1 = SerialPort(port1, baudrate)
    connect(ser1)

    ser2 = SerialPort(port2, baudrate)
    connect(ser2)

    total_time, total_bytes = send_and_receive(ser1, ser2, gcode_commands, iterations)

    print(f"Total time for {iterations * len(gcode_commands)} commands: {total_time:.4f} seconds")
    print(f"Average round-trip time: {total_time / (iterations * len(gcode_commands)):.4f} seconds")
    print(f"Total bytes sent: {total_bytes} bytes")
    print(f"Throughput: {total_bytes / total_time:.2f} bytes/second")


if __name__ == "__main__":
    main()
