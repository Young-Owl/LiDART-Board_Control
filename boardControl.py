#!/usr/bin/env python3

# Python 3.8.18
# Install packages:
# C:/Users/young/miniconda3/envs/lidart/python.exe -m pip install colorama

# UEye Coding Manual: https://www.1stvision.com/cameras/IDS/IDS-manuals/uEye_Manual/is_gamma.html

import argparse
import threading
import time
from ast import arg
from ctypes import util
from datetime import datetime
from tabnanny import verbose
from time import sleep

import cv2
import numpy as np
import serial.tools.list_ports

#from pypyueye import utils as pypy
import utils as pypy
from colorama import Fore, Style, init
from PIL import Image
from pyueye import ueye
from serial import Serial
from simple_pyueye import CameraObj as Camera

# Information declared via argparse (Global Variables; Not the best practice, but it works)
parser = argparse.ArgumentParser(description='LiDART Pulse Generator Script, responsible for the board setup, or testing it.')

# Continuous Mode was an experimental mode that was firstly used for setting the laser to be turned on continuously using the PCB.
# Deprecated since there's no need for it. The laser can be turned on using a power supply.
parser.add_argument('-c', '--continuous_mode', action="store_true", help='Defines if laser will be \'continously\' turned on.')

# Dual shot Mode takes two cameras, one with a certain exposure time and the other with a different exposure time.
parser.add_argument('-ds', '--dual_shot', action="store_true", help='Makes the camera take two shots. Used for data gathering purposes.')

# Multiple Pics Mode takes eight pictures with the same exposure time to later make an average.
parser.add_argument('-mp', '--multiple_pics', action="store_true", help='Instead of taking just one pic, takes eight to later make an average.')
args = parser.parse_args()


def get_exposure(self):
        """
        Get the current exposure.

        Returns
        =======
        exposure: number
            Current exposure.
        """
        exposure = ueye.c_double()
        pypy.check(ueye.is_Exposure(self.id, ueye.IS_EXPOSURE_CMD_GET_EXPOSURE,
                               exposure,  8))
        return exposure

def set_exposure(self, exposure):
        """
        Set the exposure.

        Returns
        =======
        exposure: number
            Real exposure, can be slightly different than the asked one.
        """
        new_exposure = ueye.c_double(exposure)
        pypy.check(ueye.is_Exposure(self.id,
                               ueye.IS_EXPOSURE_CMD_SET_EXPOSURE,
                               new_exposure, 8))
        return new_exposure

def get_pixelclock(self):
        """
        Get the current pixelclock.

        Returns
        =======
        pixelclock: number
            Current pixelclock.
        """
        pixelclock = ueye.c_uint()
        pypy.check(ueye.is_PixelClock(self.id, ueye.IS_PIXELCLOCK_CMD_GET,
                                 pixelclock, 4))
        return pixelclock

def set_pixelclock(self, pixelclock, verbose = False):
        """
        Set the current pixelclock.

        Params
        =======
        pixelclock: number
            Current pixelclock.
        """
        # Warning
        # print('Warning: when changing pixelclock at runtime, you may need to '
        #      'update the fps and exposure parameters')
        # get pixelclock range
        pcrange = (ueye.c_uint*3)()
        pypy.check(ueye.is_PixelClock(self.id, ueye.IS_PIXELCLOCK_CMD_GET_RANGE,
                                 pcrange, 12))
        pcmin, pcmax, pcincr = pcrange
        if verbose:
            print(f"Pixelclock range: [{pcmin}, {pcmax}]")
        if pixelclock < pcmin:
            pixelclock = pcmin
            print(f"Pixelclock out of range [{pcmin}, {pcmax}] and set "
                  f"to {pcmin}")
        elif pixelclock > pcmax:
            pixelclock = pcmax
            print(f"Pixelclock out of range [{pcmin}, {pcmax}] and set "
                  f"to {pcmax}")
        # Set pixelclock
        pixelclock = ueye.c_uint(pixelclock)
        pypy.check(ueye.is_PixelClock(self.id, ueye.IS_PIXELCLOCK_CMD_SET,
                                 pixelclock, 4))

def set_delay(delay):
    """
    Function responsible for converting the input delay into 4 bytes, little endian format.
    
    :param delay: The delay to be converted into bytes.
    :return lilval: The delay in bytes, little endian format.
    """
    if(delay == "Continuous"):
        return b'\x30\xFF'
    else:
        decVal = int(delay / 83.333e-9)
        hexVal = format(decVal, '04x')
        lilVal = bytes.fromhex(hexVal)[::-1]
        return lilVal
    # Output will be in little endian format
    # Ex: 100us -> 0x04B0 -> b'\xB0\x04'
    
def find_lidar():
    """
    Function responsible for finding the COM port that the LiDART Pulse Generator is connected to.
    
    :return port: The COM port that the LiDART Pulse Generator is connected to.
    """
    ser = None
        
    # Search for the COM port that the LiDART Pulse Generator is connected to
    # and open a serial connection to it. (USB Device)
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
            print("{}: {} [{}]".format(port, desc, hwid))
            if "USB Serial" in desc:
                print("Found LiDART Pulse Generator on port: {}".format(port))
                ser = Serial(port, 9600, timeout=100)
                break
    if ser is None:
        print("LiDART Pulse Generator not found.")
        return None
    else:
        return ser

def camScreenshot(cam, ser, lsrData, shotNmb, gainboost):
    ser.flushInput()
    ser.flushOutput()
    
    # Set the camera to trigger mode (Triggers the camera via PCB)
    ueye.is_SetExternalTrigger(0, ueye.IS_SET_TRIGGER_CONTINUOUS)
    print(Fore.BLUE + "Camera set as Trigger")

    traffic = ser.write(b'\x24\x01\x00\x00\x23')		        # Send a trigger event to the LiDART Pulse Generator

    # Uses modified version of the pyueye library
    # Lib\site-packages\simple_pyueye\camera.py

    # Set color mode: https://www.1stvision.com/cameras/IDS/IDS-manuals/uEye_Manual/sdk_allgemeines_farbformate.html
    cam.set_color_mode(ueye.IS_CM_MONO12)                       # Set the color mode to MONO12 (12bit grayscale)
    cam.alloc_mem(verbose = False)                              # Allocate memory for the camera after all settings have been set     
    
    frame = cam.capture_still(save = False, boost = gainboost)   # Capture a still image from the camera

    # Convert the image to match the camera's color mode and bit depth
    frame = np.squeeze(frame, axis=2)
    im = Image.fromarray((frame*(2^12)).astype(np.uint16))
    frame = np.array(im, dtype=np.uint16)
    
    # Get the current date and time (Old Method)
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H.%M.%S")
    
    height = frame.shape[0]; width = frame.shape[1] 
    
    if args.dual_shot:
        text = "LsrD: " + str(lsrData[0]) + " ; LsrW: " + str(lsrData[1]) + " ;  CamD: " + str(lsrData[2]) + " ; CamW: " + str(lsrData[3]) + " ; CamW2: " + str(lsrData[4]) + " ; GainBoost: " + str(gainboost)
    else:
        text = "LsrD: " + str(lsrData[0]) + " ; LsrW: " + str(lsrData[1]) + " ;  CamD: " + str(lsrData[2]) + " ; CamW: " + str(lsrData[3]) + " ; GainBoost: " + str(gainboost)
    cv2.putText(frame, text, (20, height - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 16)
    cv2.putText(frame, text, (20, height - 20), cv2.FONT_HERSHEY_PLAIN, 2, (2**16, 2**16, 2**16), 2)

    # =============================================== #
    # Modify these values before taking the picture so the name turns out correctly
    dataStr = "30mW_2m"
    focusStr = "_F1"
    laser = 1
    # =============================================== #
    
    if gainboost == ueye.IS_SET_GAINBOOST_ON:
        gainStr = "GB_ON"
    else:
        gainStr = "GB_OFF"
        
    if laser == 1:
        laserStr = "_Laser1"
    elif laser == 2:
        laserStr = "_Laser2"
    
    if shotNmb == 0 and args.dual_shot:
        fileStr = dataStr + "_" + str(lsrData[3]) + "_" + gainStr + laserStr
    elif shotNmb == 1 and args.dual_shot:
        fileStr = dataStr + "_" + str(lsrData[4]) + "_" + gainStr + laserStr
    elif shotNmb == 0 and not args.dual_shot:
        fileStr = dataStr + "_" + str(lsrData[3]) + "_" + gainStr 
    elif args.multiple_pics:
        fileStr = dataStr + "_" + str(lsrData[3]) + "_" + gainStr
    
    
    
    if args.dual_shot:
        cv2.imwrite("./images/" + fileStr + focusStr + "_S" + str(shotNmb)  + "_DualShot_DualLasers.png", frame)
    elif args.multiple_pics: 
        cv2.imwrite("./images/" + fileStr + focusStr + "_S" + str(shotNmb)  + "_MultiplePics.png", frame)
    else:
        cv2.imwrite("./images/" + fileStr + focusStr + "_Pulse.png", frame)

def camContinuous(cam, ser, lsrData):
    """
    Thread function whose purpose is to simply show the camera feed.
    
    :param cam: The camera object.
    :param ser: The serial connected to.
    """
       
    # In order for this to work, a modified version of the pyueye library is needed.
    # Lib\site-packages\simple_pyueye\camera.py
    # Add a new import, line 52 -> import cv2; 
    
    # Inside def continuous_capture(self), line 592     
    """
        cv2.imshow("Video", self.currContainer)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.capture = False
    """
        
    cam.install_event(ueye.IS_FRAME)
    cam.continuous_capture()
    
    # Once the user closes the camera, config the laser to send one last pulse 
    # to turn it off once done, for safety reasons.
    ser.close()
    sleep(0.1)
    ser.open()
    
    lsrW = set_delay(10e-6)			# Laser Width
    ser.flushInput()									
    ser.flushOutput()									
    ser.write(b'\x24\x06' + lsrW + b'\x23')
    sleep(0.1)
    ser.flushInput()									
    ser.flushOutput()
    ser.write(b'\x24\x01\x00\x00\x23')   
    
    sleep(1)
    print(Fore.RED + "Laser turned off!")
    print("Opening the camera again to verify it's status; Press 'q' to confirm.")
    cam.continuous_capture()

def main():
    # Colorama initialization
    init(autoreset=True)
    
    # Find the LiDART Pulse Generator USB Serial Port 
    ser = find_lidar()
    if ser is None:
        return
    
    # Setting up the camera and open it
    cam = Camera(0)
    cam.open()
    
    # Camera Gamma - Leave this to default value
    # Max: 1000, Min: 1 (Default: 100)
    cam.set_gamma(100)

    '''	Choosing the delays and times:
    # 	PIC18F2550 Clock -> 48MHz
    # 	1/48MHz = 20.833ns
    # 	(1/48MHz) * 4 = 83.333ns
    # 	83.333ns * 1200 (0x04B0) = 100us
    #	Changing the camera width to 0x04B0 -> 100us -> little endian 
    #
    # 	x -> desired delay; 
    # 	y -> dec value to write on uC;
    # 		x / 83.333ns = y
    #
    # ------------------------------------------------- #
    #
    #	  <--- camD --->|<-- camW -->|
    #					______________
    #				____|____________|____	
    #				|	|			 |	 |
    #				|	|			 |	 |
    #	  __________|___| 			 |___|___________
    #		
    #	  <- lsrD ->|<------ lsrW ------>|
    # ------------------------------------------------- # '''

    # PCB Parameters (Only needed for sincronization purposes)
    lsrDtime = 20e-6        # Laser Delay
    lsrWtime = 80e-6        # Laser Width
    camDtime = 20e-6        # Camera Delay
    camWtime = 50e-6      # Camera Width (Mininum 18us)
    
    # Variables used for the second shot, if dual_shot is enabled
    camWtime2   = 0                                     # Camera Width for the second shot, if dual_shot is enabled
    camWtimems2 = 0                                     # Camera Width (ms), if dual_shot is enabled
    camW2       = bytes.fromhex(format(0, '04x'))       # Camera Width (little endian), if dual_shot is enabled
    
    # Transform the camera width into ms (For the exposure function)
    camWtimems  = camWtime * 1000                       # Camera Width (Mininum 18us)
    
    # Data used for the laser and camera delays and widths
    lsrData     = [lsrDtime, lsrWtime, camDtime, camWtime]
    
    # Set the delays and widths for the first shot
    lsrD = set_delay(lsrDtime)			
    lsrW = set_delay(lsrWtime)			
    camD = set_delay(camDtime)			
    camW = set_delay(camWtime)			
    
    print(Fore.YELLOW + "Shot 1: " + str(lsrD) + "  " + str(lsrW) + "  " + str(camD) + "  " + str(camW))

    # Defines the camera width for the second shot, if dual_shot is enabled
    if args.dual_shot:
        camWtime2 = 100e-6
        camWtimems2 = camWtime2 * 1000
        lsrData = [lsrDtime, lsrWtime, camDtime, camWtime, camWtime2]
        camW2 = set_delay(camWtime2)
        print(Fore.YELLOW + "Shot 2: " + str(lsrD) + "  " + str(lsrW) + "  " + str(camD) + "  " + str(camW2))
    
    if not args.continuous_mode:
        if args.dual_shot:
            print(Fore.BLUE + "Script will launch on Pulse Mode with Dual Shot.")
        if args.multiple_pics:
            print(Fore.BLUE + "Script will launch on Pulse Mode with Multiple Pics.")
        else: 
            print(Fore.BLUE + "Script will launch on Pulse Mode.")
        
        if ser.isOpen():
                try:        
                    # Imported functions from 'pypyueye' library, modified to work with the 'pyueye' library          
                    set_pixelclock(cam, 474)                            # Set the pixelclock to 474MHz
                    # print(get_pixelclock(cam))                        # Get the current pixelclock
                    set_exposure(cam,camWtimems)                        # Set the exposure to "camWtime"
                    # print(get_exposure(cam))                          # Get the current exposure
                    cam.set_fps(50, verbose=False)                      # Set the camera to 50fps
                    
                    ser.flushInput()									# Flush input buffer, discarding all its contents
                    ser.flushOutput()									# Flush output buffer, aborting current output and discard all that is in buffer
                    ser.write(b'\x24\x06' + lsrD + b'\x23')				# Changing the laser delay time 
                    ser.write(b'\x24\x07' + lsrW + b'\x23')				# Changing the laser width 
                    ser.write(b'\x24\x08' + camD + b'\x23')				# Changing the camera delay time
                    ser.write(b'\x24\x09' + camW + b'\x23')				# Changing the camera width
                    
                    sleep(0.1) 
                    camScreenshot(cam, ser, lsrData, 0, ueye.IS_SET_GAINBOOST_OFF)
                    sleep(0.2)
                    camScreenshot(cam, ser, lsrData, 0, ueye.IS_SET_GAINBOOST_ON)
                    sleep(0.2)
                    
                    # If dual_shot is enabled, take a second shot (Used for data gathering purposes)
                    if args.dual_shot:
                        set_exposure(cam,camWtimems2)
                        print(get_exposure(cam))
                        
                        ser.flushInput()
                        ser.flushOutput()
                        ser.write(b'\x24\x09' + camW2 + b'\x23')				# Changing the camera width
                        
                        
                        sleep(0.1)
                        camScreenshot(cam, ser, lsrData, 1, ueye.IS_SET_GAINBOOST_OFF)
                        sleep(0.2)
                        camScreenshot(cam, ser, lsrData, 1, ueye.IS_SET_GAINBOOST_ON)
                        sleep(0.2)
                        
                    if args.multiple_pics:
                        for i in range(8):
                            sleep(0.1)
                            camScreenshot(cam, ser, lsrData, i, ueye.IS_SET_GAINBOOST_OFF)
                            sleep(0.2)
                            camScreenshot(cam, ser, lsrData, i, ueye.IS_SET_GAINBOOST_ON)
                            sleep(0.2)
                    
                     
                    cam.close(verbose=False)    # Close the camera
                    #response = ser.readline()
                    
                except Exception as err:
                    print ("Error during capture/communication: " + str(err))
        else:
            print ("Can't open serial port.")
    else:
        # Create a thread for the camera continuous capture
        threading.Thread(target = camContinuous, args = (cam,ser,lsrData,)).start()
        sleep(1)

        print(Fore.BLUE + "Script will launch on Continuous Mode.")
        if ser.isOpen():
                try:
                    ser.flushInput()									# Flush input buffer, discarding all its contents
                    ser.flushOutput()									# Flush output buffer, aborting current output and discard all that is in buffer
                    ser.write(b'\x24\x06' + lsrD + b'\x23')				# Changing the laser delay time 
                    ser.write(b'\x24\x07' + b'\x30\xFF' + b'\x23')		# Changing the laser width 
                    ser.write(b'\x24\x08' + camD + b'\x23')				# Changing the camera delay time
                    ser.write(b'\x24\x09' + camW + b'\x23')				# Changing the camera width
                    
                    sleep(0.1)
                    
                    ser.flushInput()
                    ser.flushOutput()
                    print(Fore.RED + "Starting laser!")
                    traffic = ser.write(b'\x24\x01\x00\x00\x23')		# Send a trigger event to the LiDART Pulse Generator   
                      
                except Exception as err:
                    print ("Error comunicating: " + str(err))
        else:
            print ("Can't open serial port.")
            return
        
if __name__ == '__main__':
    main()