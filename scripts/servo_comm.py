import serial
import time

SERIAL_PORT = "COM8"
BAUD_RATE = 115200

time_delay = 1

def main():

    pan = 90
    tilt = 90
    pan_gain = -10
    tilt_gain = -10

    arduino = serial.Serial(SERIAL_PORT,BAUD_RATE,timeout=1)
    time.sleep(2)
    print(f"Connected to {SERIAL_PORT}")
    arduino.write(f"{pan},{tilt}\n".encode())

    while True:
        user_input = input(": ")
        if user_input.lower() == 'q':
            break
        elif user_input == "w":
            tilt+=tilt_gain
        elif user_input == "a":
            pan-=pan_gain
        elif user_input == "s":
            tilt-=tilt_gain
        elif user_input == "d":
            pan+=pan_gain
        elif user_input =="x":
            pass
        
        arduino.write(f"{pan},{tilt}\n".encode())
        print(pan,tilt)
        time.sleep(time_delay)
        pan = tilt = 90
        arduino.write(f"{pan},{tilt}\n".encode())
            
        #time.sleep(0.1)
        #while arduino.in_waiting:
        #print(arduino.readline().decode().strip())
    arduino.close()

if  __name__ == "__main__":
    main()
