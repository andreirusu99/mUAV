"""
Ground Control Station (GCS)

Reads data from a joystick connected to the computer and sends the axis and button values
over UDP to the aircraft computer.

"""

import socket
import struct
import time

import pygame

_TAG = "Control Station"

axis = {'YAW': 0, 'THROTTLE': 1, 'ROLL': 3, 'PITCH': 4, 'TRIG': 5}
button = {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LS': 4, 'RS': 5, 'BACK': 6, 'START': 7, 'XBOX': 8, 'LDOWN': 9, 'RDOWN': 10}
hat = {'CENTER': (0, 0), 'LEFT': (-1, 0), 'RIGHT': (1, 0), 'UP': (0, 1), 'DOWN': (0, -1)}

# Main configuration
UDP_IP = "192.168.137.113"  # WiFi Vehicle IP address
# UDP_IP = "192.168.100.58"  # LAN Vehicle IP address
UDP_PORT = 51444

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cycle_Hz = 20  # loop cycle
update_rate = 1 / cycle_Hz

joystick = None


def main():
    global joystick
    print("Starting Ground Control Station...")

    try:
        pygame.init()
        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print("Started successfully!")

    except Exception as error:
        print(_TAG
              + " ERROR: No joystick connected on the computer, "
              + str(error))
        exit()

    print("Broadcasting to {} on {}".format(UDP_IP, UDP_PORT))

    while True:
        try:
            current = time.time()
            elapsed = 0

            # Joystick reading
            pygame.event.pump()

            roll = round(joystick.get_axis(axis['ROLL']), 3)
            pitch = round(joystick.get_axis(axis['PITCH']), 3)
            yaw = round(joystick.get_axis(axis['YAW']), 3)
            throttle = round(joystick.get_axis(axis['THROTTLE']), 3)

            triggers = round(joystick.get_axis(axis['TRIG']), 3)

            A = joystick.get_button(button['A'])
            B = joystick.get_button(button['B'])
            X = joystick.get_button(button['X'])
            Y = joystick.get_button(button['Y'])
            LS = joystick.get_button(button['LS'])
            RS = joystick.get_button(button['RS'])
            hat_LR, hat_UD = joystick.get_hat(0)

            message = [
                roll, pitch, yaw, throttle
                , triggers
                , A, B, X, Y, LS, RS
                , hat_LR, hat_UD
            ]

            # Print message to STDOUT
            # print("{}: {} to {}:{}".format(time.time(), message, UDP_IP, UDP_PORT))

            # Assemble message
            buf = struct.pack('>' + 'd' * len(message), *message)

            # Send message via UDP
            sock.sendto(buf, (UDP_IP, UDP_PORT))

            # Make this loop work at update_rate
            while elapsed < update_rate:
                elapsed = time.time() - current

        except Exception as e:
            print(e)


if __name__ == "__main__":

    try:

        main()

    except Exception as error:
        pass
