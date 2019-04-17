
#Connect to port
import serial
import time
import getch #make sure you install getch: pip install getch
from collections import namedtuple
ser = serial.Serial('/dev/ttyS0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
print("Connected to: " + ser.portstr);	

#Initialize Structure
#controlStruct = namedTuple("controlStruct", "velocity theta mode");
class controlStruct():
	def __init__(velocity, theta, mode):
		self.velocity = velocity
		self.theta = theta
		self.mode = mode
#Error Handling

while True:
	#Search for Terminal Input (blocking)
	#Parse terminal input, get a velocity and a theta
	inputAngle = 0
	inputVelocity = 0
	inputMode = -1
	print("Input Mode: ")
	while inputMode = -1;
		inputMode = getch.getche()
		if inputMode != 0 or inputMode != 1:
			inputMode = -1
	print("Input Angle: ")
	while inputAngle = 0:
		inputAngle = getch.getche()
		if inputAngle!= 'h' or inputAngle != 'j' or inputAngle != 'k' or inputAngle != 'l':
			inputAngle = 0
	print("Input Velocity: ")
	while inputVelocity = 0:
		inputVelocity = getch.getche()
		if inputVelocity != 'w' or inputVelocity != 'a' or inputVeloocity != 's' or inputVelocity != 'd':
			inputVelocity = 0
	
	#Set structure to velocity and theta
	mystruct = controlStructure(inputVelocity, inputTheta, inputMode)
	#Send structure
	wer.write(mystruct)