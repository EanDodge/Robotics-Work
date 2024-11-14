import pygame
import socket
import struct
import time
import numpy as np
import cv2 as cv
 
# Create a black image
img = np.zeros((500,500,3), np.uint8)

# Initialize Pygame
pygame.init()
pygame.joystick.init()

# Set up the UDP socket
#server_address = ("127.0.0.1", 8888)
server_address = ("192.168.2.180", 8888)
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
bufferSize          = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


x = 0
y = 0
MyX = x
MyY = y

#Initialize the joystick
if pygame.joystick.get_count() == 0:
    print("No joystick detected")
    #quit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

try:
    print("Joystick Name:", joystick.get_name())

    while True:
        # Handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit()


        # Get joystick input
        axis_x = joystick.get_axis(0)
        axis_y = joystick.get_axis(1)
        buttons = joystick.get_numbuttons()
        button_states = [joystick.get_button(i) for i in range(buttons)]

        
        # Pack joystick data into a struct
        data = str.encode(f"joy {int(axis_x * 100)} {int(axis_y * 100)} {button_states}")
#        print(data)

        # Send joystick data over UDP
        UDPClientSocket.sendto(data, server_address)


        try:
            UDPClientSocket.settimeout(1.0)

            msgFromServer = -1

            msgFromServer = UDPClientSocket.recvfrom(bufferSize)

            UDPClientSocket.settimeout(None)
            #print(msgFromServer)

            msg = "Message from Server {}".format(msgFromServer[0])
            decoded_string = msgFromServer[0].decode("utf-8")
            print(decoded_string)
            str1 = msgFromServer[0].decode("utf-8").split( )
            if len(str1) == 3 and str1[0] == "pose" :
                x = int(str1[1])
                y = int(str1[2])

            img = cv.line(img, (MyX,MyY), (x,y), color=(0, 255, 255), thickness=5)

            MyX = x
            MyY = y
            cv.imshow('pyjoy cv2',img)
            cv.waitKey(3)
        except:
            pass

finally:
    pygame.quit()
    udp_socket.close()