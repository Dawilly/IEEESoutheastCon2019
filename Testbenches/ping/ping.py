import serial, time

def readLine(remote_sys):
    while 1:
        if (remote_sys.in_waiting > 0):
            line_r = remote_sys.readline()
            return line_r

# initialize serial communication
arduino = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(1)

# wait for remote system to set-up
print("Waiting for remote system . . . " + readLine(arduino))

# write message
line_w = raw_input("Message: ")
line_w += '\n'
arduino.write(line_w.encode())

# read message
print("Response: " + readLine(arduino))
