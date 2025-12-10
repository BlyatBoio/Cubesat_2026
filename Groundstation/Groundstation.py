import os
import sys
import serial
import time as clock
from PyQt5 import uic
from PyQt5 import QtGui
from PyQt5 import QtCore
from datetime import datetime
import serial.tools.list_ports

import threading

try:
    log_file = "log.txt"
    if not os.path.isfile(log_file):
        with open(log_file, "w") as file:
            pass

    def log(string, includeTime=True):
        if(includeTime):
            print("["+datetime.now().strftime("%H:%M:%S")+"] "+string)
            with open(log_file, "a") as logFile:
                logFile.write("["+datetime.now().strftime("%H:%M:%S")+"] " + string+"\n")  # Write content to the file
        else:
            print(string)
            with open(log_file, "a") as logFile:
                logFile.write(string+"\n")  # Write content to the file
         
    log("", False)   
    log("---Welcome To The Groundstation---", False)

    class Transciever:
        def __init__(self):
            log("", False)
            log("Initialized Transciever")
            self.port = None
            self.serial = None
            self.isConnected = False
        
        def connect(self):
            log("Attempting To Connect")
            port = None
            
            available_ports = list(serial.tools.list_ports.comports())
            
            log("Collected "+str(len(available_ports))+" Ports")

            for checkPort in available_ports:
                log("Checking Port: " + str(checkPort))
                if "USB-Enhanced-SERIAL" in str(checkPort):
                    log("Set Port To: " + str(checkPort))
                    port = checkPort.device     
                    
            if port is not None:
                self.serial = serial.Serial(str(port), 9600, timeout=0.5, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
                log("Assigned Port: " + str(port) + "\n")
                self.port = port
                self.isConnected = True
                    
            else:
                log("No Acceptable Port Connected: Trying again\n")
        
        def sendCommand(self, command):
            if(not self.isConnected): log("No Port Connected!")
            else:
                self.serial.write(str(command).encode("ascii"))
                log("Sent: " + str(command))
                
        def readData(self):
            return self.serial.read(960)

    radio = Transciever()
    
    def runCommand(cmd):
        Fcmd = cmd.lower().replace(" ", "")
        if(Fcmd[0:5] == "clear"):
            os.system('cls');
        else:
            radio.sendCommand(cmd)

    def connectRadio():
        while not radio.isConnected:
            radio.connect()

    def runCommandInput():
        while True:
            command = input("")
            runCommand(command)

    def runRadioReciever():
        while True:
            packet = str(radio.serial.read(960).decode("ascii"))
            if packet != "": 
                packets = packet.split("end_msg")
                for p in packets :
                    if p != "":
                        p = decodeData(p)
                        log("Recieved: " + p, False)
            clock.sleep(1)
    
    def decodeData(packet):
        newPacket = ""
        packet = packet.lower()
        splitPacket = packet.split(" ") # start at index 1 because index 0 is gps
        if splitPacket[0] == "gps":
            log("Recieved GPS Packet")
            newPacket += "\nLatitude: {} Degress\nLongitude: {} Degrees\nAltitude: {} m\n Speed: {} m/s\nSatelites: {}\nDOP: {}".format(
                splitPacket[1],
                splitPacket[2],
                splitPacket[3],
                splitPacket[4],
                splitPacket[5],
                splitPacket[6])
        elif splitPacket[0] == "alt":
            log("Recieved Altimeter Packet")
            newPacket += "\nAltitude: {} m\nTemperature: {} C\nPressure: {} Kpa\nHumidity {} %\nGas: {} Ohms".format(
                splitPacket[1],
                splitPacket[2],
                splitPacket[3],
                splitPacket[4],
                splitPacket[5])
        elif splitPacket[0] == "imu":
            log("Recieved IMU Packet")
            newPacket += "\nAcceleration X: {} m/s^2\nAcceleration Y: {} m/s^2\nAcceleration Z: {} m/s^2\nRotation X: {} Rad/s\nRotation Y: {} Rad/s\nRotation Z: {} Rad/s".format(
                splitPacket[1],
                splitPacket[2],
                splitPacket[3],
                splitPacket[4],
                splitPacket[5],
                splitPacket[6])
        elif splitPacket[0] == "mag":
            log("Recieved Magnometer Packet")
            newPacket += "\nMagnetic Strength X: {} uT\nMagnetic Strength Y: {} uT\nMagnetic Strength Z: {} uT".format(
                splitPacket[1],
                splitPacket[2],
                splitPacket[3])
        elif splitPacket[0] == "pow":
            log("Recieved Power Packet")
            newPacket += "\nPower Draw Voltage: {:.3f} V\nPower Draw Current: {:.3f} A\nPower Draw Wattage: {:.3f} W".format(
                splitPacket[1],
                splitPacket[2],
                splitPacket[3])
            newPacket += "\nSolar 1 Voltage: {:.3f} V\nSolar 1 Current: {:.3f} A\nSolar 1 Wattage: {:.3f} W".format(
                splitPacket[4],
                splitPacket[5],
                splitPacket[6])
            newPacket += "\nSolar 2 Voltage: {:.3f} V\nSolar 2 Current: {:.3f} A\nSolar 2 Wattage: {:.3f} W".format(
                splitPacket[7],
                splitPacket[8],
                splitPacket[9])
            newPacket += "\nSolar 3 Voltage: {:.3f} V\nSolar 3 Current: {:.3f} A\nSolar 3 Wattage: {:.3f} W".format(
                splitPacket[10],
                splitPacket[11],
                splitPacket[12])
            newPacket += "\nBattery Voltage: {} V\nBattery Percentage: {} %".format(
                splitPacket[13],
                splitPacket[14])
        else: 
            newPacket = packet
        
        return newPacket
            
    radioConnectThread = threading.Thread(target=connectRadio)
    commandRunnerThread = threading.Thread(target=runCommandInput)
    radioInputThread = threading.Thread(target=runRadioReciever)
    log("Initialized Threads")
    
    radioConnectThread.start()
    radioConnectThread.join()
    commandRunnerThread.start()
    radioInputThread.start()
    log("Started Threads")
except:
    log("Critical Error")