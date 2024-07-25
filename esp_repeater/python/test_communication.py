import serial
import time
from typing import List


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

def send_and_receive_batch(ser1: SerialPort, ser2: SerialPort, commands: List[str], iterations: int):
    total_time = 0
    total_bytes = 0
    st = time.time()
    read_data = []
    send_data = []
    idx = 0
    for i in range(iterations):
        for command in commands:
            command = f"{str(idx).zfill(5)} {command}"
            ser1.write(command)
            send_data.append(command)

            # Read the response from ser2
            response = ser2.read()
            read_data.append(response)

            total_bytes += len(command)
            idx += 1
    read_data = ''.join(read_data)
    send_data = ''.join(send_data)
    print(f"all data sent in {time.time() - st} seconds")
    print(f"Length of sent data: {len(send_data)}")
    print(f"Length of read data: {len(read_data)}")

    # Ensure all data has been received
    last_dl = len(send_data) - len(read_data)
    ntimes = 0
    while len(read_data) < len(send_data):
        response = ser2.read()
        if response:
            read_data += response
        dl = len(send_data) - len(read_data)
        if dl == last_dl:
            if ntimes < 10:
                ntimes += 1
                continue
            print(f"Timeout: {len(send_data) - len(read_data)}")
            print(f"Send data: {send_data[-100:]} \nRead data: {read_data[-100:]}")
            break
        else:
            ntimes = 0
        last_dl = dl
    print(f"all data received in {time.time() - st} seconds")
    et = time.time()

    # Compare sent and received data
    if read_data.strip() != send_data.strip():
        print(f"Mismatch detected:")
        for idx in range(len(send_data)):
            if idx >= len(read_data):
                print(f"reached end index of read_data: {idx}")
                break
            if send_data[idx] != read_data[idx]:
                around = 20
                print(f"found index of mismatch {idx}: Sent '{send_data[idx-around:idx+around]}', Received '{read_data[idx-around:idx+around]}'")
                break
        print(f"Send data: {send_data[-100:]} \nRead data: {read_data[-100:]}")
    else:
        print("Data is correct")

    ser1.close()
    ser2.close()

    return et - st, total_bytes


def connect(serial_port):
    dta = serial_port.read()
    while dta != '(ok)':
        print(dta)
        dta = serial_port.read()


def main():
    baudrate = 115200
    boudrate1 = 460800
    # Serial port settings
    port1 = 'COM3'  # Adjust as necessary
    port2 = 'COM4'  # Adjust as necessary

    # G-code commands to test
    gcode_commands = ["G0 X10 Y10;", "G1 X20 Y20;", "G28;", "M114;", "M105;"]

    # Number of iterations
    iterations = 1000
    ser1 = SerialPort(port1, baudrate=boudrate1)
    connect(ser1)

    ser2 = SerialPort(port2, baudrate)
    connect(ser2)

    total_time, total_bytes = send_and_receive_batch(ser1, ser2, gcode_commands, iterations)

    print(f"Total time for {iterations * len(gcode_commands)} commands: {total_time:.4f} seconds")
    print(f"Average round-trip time: {total_time / (iterations * len(gcode_commands)):.4f} seconds")
    print(f"Total bytes sent: {total_bytes} bytes")
    print(f"Throughput: {total_bytes / total_time:.2f} bytes/second")

if __name__ == "__main__":
    main()
