import serial
import time
 
# Establish a serial connection with the Arduino
arduino = serial.Serial(port='COM4', baudrate=57600, timeout=.1)
 
def send_angles(theta_R, theta_W, theta_G1, theta_G2):
    # Format the angles as a comma-separated string
    angles = f"{theta_R},{theta_W},{theta_G1},{theta_G2}"
    
    # Send the data to the Arduino
    arduino.write(bytes(angles, 'utf-8'))
    print(f"Sent to Arduino: {angles}")
 
def get_user_input():
    while True:
        try:
            # Get user inputs
            theta_R = float(input("Enter theta_R: "))
            theta_W = float(input("Enter theta_W: "))
            theta_G1 = float(input("Enter theta_G1: "))
            theta_G2 = float(input("Enter theta_G2: "))
 
            # Send the angles to the Arduino
            send_angles(theta_R, theta_W, theta_G1, theta_G2)
        
        except ValueError:
            print("Invalid input. Please enter numerical values.")
        
        # Small delay to prevent spamming the Arduino
        time.sleep(0.1) # in seconds
 
# Start the input loop
get_user_input()
