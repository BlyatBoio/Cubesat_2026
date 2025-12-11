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
    # Open or create log file
    log_file = "log.txt"
    if not os.path.isfile(log_file):
        with open(log_file, "w") as file:
            pass
        
    # Open or create error file
    error_file = "error.txt"
    if not os.path.isfile(error_file):
        with open(error_file, "w") as file:
            pass
    
    
    # colors to write to the terminal in
    os.system('color')
    errColor = '\033[91m'
    sendTextColor = '\033[92m'
    inTextColor = '\033[94m'
    timeColor = '\033[97m'
    resetColor = '\033[0m'
    
    def log(string, includeTime=True, doTextColor=True):
        # include time adds [xx:xx:xx] before the given string
        if includeTime :
            time = "["+datetime.now().strftime("%H:%M:%S")+"]"
            
            if doTextColor: print(f"{timeColor}{time} {sendTextColor}{string}{resetColor}") # print string to terminal
            else: print(f"{time} {string}")
            
            with open(log_file, "a") as logFile:
                logFile.write(time+string+"\n")  # Write content to the file
        else:
            if doTextColor: print(f"{sendTextColor}{string}{resetColor}") # print string to terminal
            else: print(string)
            
            with open(log_file, "a") as logFile:
                logFile.write(string+"\n")  # Write content to the file
         
    def error(string):
        time = "["+datetime.now().strftime("%H:%M:%S")+"]"
        print(f"{timeColor}{time} {errColor}{string}{resetColor}") # print string to terminal
        
        with open(error_file, "a") as errorFile:
            errorFile.write(time+string+"\n")  # Write content to the file
    # Welcome text in terminal     
    log("", False)   
    log("---Welcome To The Groundstation---", False)

    class Transciever:
        def __init__(self):
            self.port = None # serial port device
            self.serial = None # Serial interface
            self.isConnected = False # Boolean to determine whether it can read or send data
            log("", False)
            log("Initialized Transciever")
        
        def connect(self):
            try:
                log("Attempting To Connect")

                port = None
                
                # gather all availible ports
                available_ports = list(serial.tools.list_ports.comports())
                log("Collected "+str(len(available_ports))+" Ports")

                # Check all ports for a propper string
                for checkPort in available_ports:
                    log("Checking Port: " + str(checkPort))
                    if "USB-Enhanced-SERIAL" in str(checkPort):
                        # If the port has the propper tag, set it
                        log("Set Port To: " + str(checkPort))
                        port = checkPort.device     
                        
                if port is not None:
                    # If a port was assigned, assign it to the Serial device
                    self.serial = serial.Serial(str(port), 9600, timeout=0.5, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
                    log("Assigned Port: " + str(port) + "\n")
                    self.port = port
                    self.isConnected = True
                        
                else:
                    log("No Acceptable Port Connected: Trying again\n")
            except:
                error("Error Connecting To Transciever Port")
        
        def sendCommand(self, command):
            try:
                if(not self.isConnected): log("No Port Connected!")
                else:
                    self.serial.write(str(command).encode("ascii"))
                    log("Sent: " + str(command))
            except:
                error("Error sending command")
                
        def readData(self):
            try:
                return self.serial.read(960)
            except:
                error("Error reading data")

    radio = Transciever()
    
    def runCommand(cmd):
        try:
            Fcmd = cmd.lower().replace(" ", "")
            if(Fcmd[0:5] == "clear"):
                os.system('cls');
            else:
                radio.sendCommand(cmd)
        except:
            error("Error Running Command")

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
                        if "ERROR" not in p: log(f"{timeColor}Recieved: {inTextColor}{p}")
                        else: error("Recieved " + p)
            clock.sleep(1)
    
    def decodeData(packet):
        try:
            newPacket = ""
            savePacketCasing = packet
            packet = packet.lower()
            splitPacket = packet.split(" ") # start at index 1 because index 0 is gps
            if splitPacket[0] == "gps":
                newPacket += "{}\n----------\n{}GPS Packet:\nLatitude: {} Degress\nLongitude: {} Degrees\nAltitude: {} m\n Speed: {} m/s\nSatelites: {}\nDOP: {}\n{}----------".format(
                    timeColor,
                    inTextColor,
                    splitPacket[1],
                    splitPacket[2],
                    splitPacket[3],
                    splitPacket[4],
                    splitPacket[5],
                    splitPacket[6],
                    timeColor)
            elif splitPacket[0] == "alt":
                newPacket += "{}\n----------\n{}Altimeter Packet:\nAltitude: {} m\nTemperature: {} C\nPressure: {} Kpa\nHumidity {} %\nGas: {} Ohms\n{}----------".format(
                    timeColor,
                    inTextColor,
                    splitPacket[1],
                    splitPacket[2],
                    splitPacket[3],
                    splitPacket[4],
                    splitPacket[5],
                    timeColor)
            elif splitPacket[0] == "imu":
                newPacket += "{}\n----------\n{}IMU Packet:\nAcceleration X: {} m/s^2\nAcceleration Y: {} m/s^2\nAcceleration Z: {} m/s^2\nRotation X: {} Rad/s\nRotation Y: {} Rad/s\nRotation Z: {} Rad/s\n{}----------".format(
                    timeColor,
                    inTextColor,
                    splitPacket[1],
                    splitPacket[2],
                    splitPacket[3],
                    splitPacket[4],
                    splitPacket[5],
                    splitPacket[6],
                    timeColor)
            elif splitPacket[0] == "mag":
                newPacket += "{}\n----------\n{}Magnometer Packet:\nMagnetic Strength X: {} uT\nMagnetic Strength Y: {} uT\nMagnetic Strength Z: {} uT\n{}----------".format(
                    timeColor,
                    inTextColor,
                    splitPacket[1],
                    splitPacket[2],
                    splitPacket[3],
                    timeColor)
            elif splitPacket[0] == "pow":
                newPacket += "{}\n----------\n{}Power Packet:\nPower Draw Voltage: {:.3f} V\nPower Draw Current: {:.3f} A\nPower Draw Wattage: {:.3f} W".format(
                    timeColor,
                    inTextColor,
                    splitPacket[1],
                    splitPacket[2],
                    splitPacket[3],
                    timeColor)
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
                newPacket += "\nBattery Voltage: {} V\nBattery Percentage: {} %\n{}----------".format(
                    splitPacket[13],
                    splitPacket[14])
            elif splitPacket[0] == "err":
                newPacket += "ERROR: " + packet[4:]
            else: 
                newPacket = savePacketCasing
            
            return newPacket
        except:
            error("Error Decoding Packet")

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
    error("Critical Error")