#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial

radio = serial.Serial('/dev/tty.usbserial') # TODO: change this to command line argument

pshyk_enabled = 0

def getch():
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

print "Press q for exit\n"
print "Press a, s, w, d for movement and spacebar to paint\n"

try:
    while True:
        key = getch()
        print "Key pressed is " + key
        if   key == 'a':
             radio.write("\xA1")
        elif key == 'd':
            radio.write("\xA2")
        elif key == 'w':
            radio.write("\xA3")
        elif key == 's':
            radio.write("\xA4")
        elif key == ' ':
            if( not pshyk_enabled ):
                radio.write("\x3F")
                pshyk_enabled = 1
                print "paint enabled"
            else:
                radio.write('a')
                pshyk_enabled = 0
                print "paint disabled"
        elif key == 'q':
            break  # Exit the while loop

except KeyboardInterrupt:
    pass
finally:
    radio.close()





    
