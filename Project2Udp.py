import socket
import sys
from ctypes import *

velocity = 0
phi = 0

UDP_IP = "192.168.43.35"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect((UDP_IP, UDP_PORT))

class cmdPacket(Structure):
    _fields_ = [("vel", c_double), ("theta", c_double), ("mode", c_int)]
    
class returnPacket(Structure):
    _fields_ = [("x", c_double), ("y", c_double), ("head", c_double)]

while True:
    key = raw_input("Input: ")
    
    if(key == 'w'):
        velocity += 75 
        sock.send(cmdPacket(velocity, phi, 1))
    elif(key == 'a'):
        velocity -= 50
        if(velocity < 0):
            velocity = 0
        sock.send(cmdPacket(velocity, phi, 1))
    elif(key == 's'):
        phi -= 45
        sock.send(cmdPacket(velocity, phi, 1))
    elif(key == 'd'):
        phi += 45
        sock.send(cmdPacket(velocity, phi, 1))
    elif(key == 'i'):
        velocity = 0
        phi = 0
        sock.send(cmdPacket(velocity, phi, 2))
    elif(key == 'j'):
        velocity = 0
        phi = 270
        sock.send(cmdPacket(velocity, phi, 2))
    elif(key == 'k'):
        velocity = 0
        phi = 180
        sock.send(cmdPacket(velocity, phi, 2))
    elif(key == 'l'):
        velocity = 0 
        phi = 90
        sock.send(cmdPacket(velocity, phi, 2))
    elif(key == ' '):
        sock.send(cmdPacket(velocity, phi, 0))
        buffer = sock.recv(sizeof(returnPacket))
        tempPacket = returnPacket.from_buffer_copy(buffer)
        for field_name, field_type in tempPacket._fields_:
            print field_name, getattr(tempPacket, field_name)
    elif(key == 'c'):
        velocity = 0
        sock.send(cmdPacket(velocity, phi, 3))
    elif(key == 'r'):
        sock.send(cmdPacket(velocity,phi,4))
    else:
        print("Invalid Input")
