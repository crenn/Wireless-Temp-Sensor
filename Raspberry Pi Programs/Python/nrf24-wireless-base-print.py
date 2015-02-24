#!/usr/bin/python
#
# Establish a connection to the remote sensor and print converted temperature
# values
#

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
from lib_nrf24 import NRF24
import time
import spidev
from struct import *
from array import *


pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]

radio2 = NRF24(GPIO, spidev.SpiDev())
#Setup the nRF24 on SPI CE0 and CE pin
radio2.begin(0, 18)

radio2.setRetries(3,3)

radio2.setPayloadSize(32)
radio2.setChannel(0x02)
radio2.setDataRate(NRF24.BR_2MBPS)
radio2.setPALevel(NRF24.PA_MAX)
radio2.setCRCLength(NRF24.CRC_8)

radio2.setAutoAck(True)
radio2.enableDynamicPayloads()
radio2.enableAckPayload()

radio2.openWritingPipe(pipes[0])
radio2.openReadingPipe(0, pipes[0])

radio2.startListening()
radio2.stopListening()

radio2.printDetails()

radio2.startListening()

c=0
awake=0
req_buf = "RC"
cal = []

AC5 = 0
AC6 = 0
MC = 0
MD = 0

#Setup the nRF24 IRQ pin
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
timeawake = time.time()
printed = 0
while True:
    pipe = [0]  
    try:
        while not radio2.available(pipe):
           if (awake != 0):
               if (printed == 0):
                   print("Waiting for a reply")
                   printed = 1
               if ((time.time() - timeawake) >= 5):
                   print ("Remote sensor not likely to be awake now")
                   awake = 0
                   printed = 0
               time.sleep(0.050)
           else:
               print ("Waiting for the awake interrupt")
               GPIO.wait_for_edge(17, GPIO.FALLING)
    except KeyboardInterrupt:  
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
        print ("\rCleaned up GPIO and quitting\r\n")
        quit()

    printed = 0
    recv_buffer = []
    radio2.read(recv_buffer, radio2.getDynamicPayloadSize())
    print ("Received:") ,
    print (recv_buffer)
    if (awake == 0):
        recv_buffer = str(bytearray(recv_buffer))
        if (recv_buffer == "AW"):
            print ("Remote sensor is awake!")
            awake = 1;
            radio2.stopListening()
            if (req_buf[-1] == 'C'):
                print("Requesting Calibration Values")
            elif (req_buf[-1] == 'C'):
                print("Requesting Temperature Values")
            print(radio2.write(req_buf));
#            print ("Loaded payload reply:"),
#            print (req_buf)
#            print (radio2.whatHappened())
            radio2.startListening()
            timeawake = time.time()
    else:
        if (chr(recv_buffer[0]) == 'C'):
            cal = recv_buffer
            [AC5, AC6, MC, MD] = unpack_from('>hhhh', array('B', cal), 1)
            print([AC5, AC6, MC, MD])
            if (c == 0):
                req_buf = "RT"
                c = 1;
                print ("Switching to request temp");
                radio2.stopListening()
                print(radio2.write(req_buf));
#                print ("Loaded payload reply:"),
#                print (req_buf)
#                print (radio2.whatHappened())
                radio2.startListening()
        elif (chr(recv_buffer[0]) == 'T'):
            UT = 0
            X1 = 0
            X2 = 0
            B5 = 0
            Temp = 0
            ActTemp = 0
            values = ((len(recv_buffer) - 1) / 2)
            for i in range (0, values):
                UT = ((recv_buffer[(i*2)+1] << 8) | recv_buffer[(i*2)+2]);
                X1 = ((UT - AC6) * AC5) >> 15;
                X2 = int(float((MC << 11) / (X1 + MD)));
                B5 = X1 + X2;
                Temp = (B5 + 8) >> 4;
#                print([UT, X1, X2, B5, Temp])
                ActTemp = float(Temp) / 10;
                print ("True Temp is : %.2foC" % ActTemp)
#            print (radio2.whatHappened())
            print ("Got temp, the remote sensor is now asleep")
            awake = 0;

GPIO.cleanup()           # clean up GPIO on normal exit
