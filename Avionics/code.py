import board
import busio
import storage
import analogio
import digitalio
import time as clock
import microcontroller

import adafruit_gps
import adafruit_sdcard
import adafruit_bme680
import adafruit_max1704x
from adafruit_lis3mdl import LIS3MDL
import adafruit_pcf8591.pcf8591 as PCF
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_pcf8591.analog_in import AnalogIn
from adafruit_pcf8591.analog_out import AnalogOut
from adafruit_datetime import datetime, date, time
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

try:
    class GPS:
        def __init__(self):
            self.interface = adafruit_gps.GPS(busio.UART(board.GP16,board.GP17,baudrate=9600,timeout=10),debug=False)
        
        def __repr__(self):
            return ("Latitude:{:.6f}"+
                    "Longitude:{:.6f}"+
                    "Altitude:{:.2f}"+
                    "Speed:{:.2f}"+
                    "Satelites:{}"+
                    "DOP:{}").format(
                        self.interface.latitude,
                        self.interface.longitude,
                        self.interface.altitude_m,
                        self.interface.speed_knots*(463/900)
                    )
    
    class LED:
        def __init__(self, boardPin):
            self.boardPin = boardPin
            self.interface = digitalio.DigitalInOut(self.boardPin)
            self.interface.direction = digitalio.Direction.OUTPUT
            self.value = False;
    
        def turnOn(self):
            self.value = True
            self.interface.value = True
        
        def turnOff(self):
            self.value = False
            self.interface.value = False

        def toggle(self):
            self.value = (not self.value)
            self.interface.value = self.value
        
    class Tranciever:
        def __init__(self):
            # Establish output Pins
            M0 = digitalio.DigitalInOut(board.GP6)
            M1 = digitalio.DigitalInOut(board.GP7)
            
            M0.direction = digitalio.Direction.OUTPUT
            M1.direction = digitalio.Direction.OUTPUT
            
            M0.value = False
            M1.value = False
            
            self.interface = busio.UART(board.GP8,board.GP9,baudrate=9600,timeout=0.1)

        def sendString(self, string):
            self.interface.write(string.encode("ascii"))
        
        def sendBytes(self, bytes):
            self.interface.write(bytes)
        
        def readIncoming(self):
            return self.interface.read(960)
    
    def processCommand():
        inString = str(radio.readIncoming())
        
        if(inString is not "None"):
            receiveLED.turnOn()
        else:
            return
        
        inString.lower()

        if "ping" in inString:
            radio.sendString("pong")
        elif "get" in inString:
            radio.sendString("Data Example: 100")
        
            
    def startupLightshow():
        gpsLED.turnOn()
        clock.sleep(0.1)
        transmitLED.turnOn()
        clock.sleep(0.1)
        receiveLED.turnOn()
        clock.sleep(0.1)
        processLED.turnOn()
        clock.sleep(0.1)
        errorLED.turnOn()
        clock.sleep(0.1)        
        gpsLED.turnOff()
        clock.sleep(0.1)
        transmitLED.turnOff()
        clock.sleep(0.1)
        receiveLED.turnOff()
        clock.sleep(0.1)
        processLED.turnOff()
        clock.sleep(0.1)
        errorLED.turnOff()
    
    imbeddedLED = LED(board.GP25)
    processLED = LED(board.GP21)    
    gpsLED = LED(board.GP22)
    transmitLED = LED(board.GP20)
    receiveLED = LED(board.GP18)
    errorLED = LED(board.GP19)
    radio = Tranciever()
    startupLightshow()

    while True:
        processCommand()
        clock.sleep(2)
        processLED.toggle()
        receiveLED.turnOff()
    
except:
    print("Critical Error Occured")
