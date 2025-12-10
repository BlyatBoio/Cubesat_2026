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
    sdcard = adafruit_sdcard.SDCard(busio.SPI(board.GP10,MOSI=board.GP11,MISO=board.GP12),digitalio.DigitalInOut(board.GP13))
    vfs = storage.VfsFat(sdcard)
    storage.mount(vfs, "/sd")
    I2C0 = busio.I2C(board.GP1,board.GP0,frequency=10000)
    UART0 = busio.UART(board.GP16,board.GP17,baudrate=9600,timeout=10)

    doSendGps = False
    doSendAlt = False
    doSendImu = False
    doSendMag = False
    doSendPow = False
    
    class GPS:
        def __init__(self):
            try:
                self.interface = adafruit_gps.GPS(UART0,debug=False)
                
                self.interface.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0') #Issues parameters to GPS
                self.interface.send_command(b'PMTK220,1000')
            except:
                radio.sendError("Failed To Setup GPS" )
        
        def getData(self):
            try:
                self.interface.update()
                return "GPS {:.6f} {:.6f} {:.2f} {:.2f} {} {}".format(
                    self.interface.latitude,
                    self.interface.longitude,
                    self.interface.altitude_m,
                    self.interface.speed_knots*(463/900),
                    self.interface.satellites,
                    self.interface.horizontal_dilution)
            except:
                return "err Failed To Send GPS Data"

    class Altimeter:
        def __init__(self):
            try:
                self.interface = adafruit_bme680.Adafruit_BME680_I2C(I2C0, address=0x77)

                self.interface.sea_level_pressure = 1013.25
                
                self.interface.pressure_oversampling = 8
                self.interface.temperature_oversampling = 2
            except:
                radio.sendError("Failed To Setup Altimeter ")
        
        def getData(self):
            return "ALT {:.2f} {:.2f} {:.2f} {:.2f} {:.0f}".format(
                self.interface.altitude,
                self.interface.temperature,
                self.interface.pressure,
                self.interface.relative_humidity,
                round(self.interface.gas))
            
    class IMU:
        def __init__(self):
            try:
                self.interface = LSM6DSOX(I2C0,address=0x6a)
            except:
                radio.sendError("Failed To Setup IMU ")
        def getData(self):
            return "IMU {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}".format(
                self.interface.acceleration[0],
                self.interface.acceleration[1],
                self.interface.acceleration[2],                
                self.interface.gyro[0],
                self.interface.gyro[1],
                self.interface.gyro[2])
                
    class Magnometer:
        def __init__(self):
            try:
                self.interface = LIS3MDL(I2C0,address=0x1c)
            except:
                radio.sendError("Failed To Setup Magnometer ")
        def getData(self):
            return "MAG {:.2f} {:.2f} {:.2f}".format(
                self.interface.magnetic[0],
                self.interface.magnetic[1],
                self.interface.magnetic[2])              
                
    class Power:
        def __init__(self):
            try:
                self.powerDrawInterface = INA219(I2C0,addr=0x45)

                self.powerDrawInterface.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.powerDrawInterface.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.powerDrawInterface.bus_voltage_range = BusVoltageRange.RANGE_16V
                
                self.solar1Inteface = Solar(0x40)
                self.solar2Inteface = Solar(0x41)
                self.solar3Inteface = Solar(0x44)
                
                self.batteryInterface = adafruit_max1704x.MAX17048(I2C0,address=0x36)
            except:
                radio.sendError("Failed To Setup Power ")
        def getData(self):
            try:
                return "POW {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.2f} {:.2f}".format(
                    self.powerDrawInterface.bus_voltage,
                    self.powerDrawInterface.current / 1000,
                    abs(self.powerDrawInterface.bus_voltage * (self.powerDrawInterface.current / 1000)),
                    0,0,0,0,0,0,0,0,0,
                    # Currently No Solar Panel Data To Send 
                    #self.solar1Inteface.bus_voltage,
                    #self.solar1Inteface.current / 1000,
                    #abs(self.solar1Inteface.bus_voltage * (self.solar1Inteface.current / 1000)),
                    #self.solar2Inteface.bus_voltage,
                    #self.solar2Inteface.current / 1000,
                    #abs(self.solar2Inteface.bus_voltage * (self.solar2Inteface.current / 1000)),
                    #self.solar3Inteface.bus_voltage,
                    #self.solar3Inteface.current / 1000,
                    #abs(self.solar3Inteface.bus_voltage * (self.solar3Inteface.current / 1000)),
                    self.batteryInterface.cell_voltage,
                    self.batteryInterface.cell_percent)
            except:
                return "Failed To Send Power Data"

    class Solar:
        def __init__(self, adress):
            try:
                self.interface = INA219(I2C0,addr=adress)
                
                self.interface.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.interface.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.interface.bus_voltage_range = BusVoltageRange.RANGE_16V
            except:
                radio.sendError("Failed To Setup Solar ")
    
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
            self.interface.write((string+"end_msg").encode("ascii"))
        
        def sendError(self, string):
            self.interface.write(("err "+string+"end_msg").encode("ascii"))
        
        def sendBytes(self, bytes):
            self.interface.write(bytes)
        
        def readIncoming(self):
            return self.interface.read(960)
    
    def processCommand():
        
        global doSendGps
        global doSendAlt
        global doSendImu
        global doSendMag
        global doSendPow
        
        inString = str(radio.readIncoming())
        
        if(inString is not "None"):
            receiveLED.turnOn()
        else:
            return
        
        inString = inString.lower()
        inString = inString[2:] # remove(b')
        inString = inString.replace(" ", "")
        inString = inString.replace("'", "")

        #radio.sendString("Recieved Command: " + inString + "\n")
        try:
            if inString[0:4] is "ping":
                radio.sendString("pong")
            elif inString[0:6] is "toggle":
                inString = inString[6:]
                if inString[0:4] is "data":
                    inString = inString[4:]
                    #setValue = True if inString[3:7] is "true" else False
                    if inString[0:3] is "gps":
                        doSendGps = not doSendGps
                        radio.sendString(("Now" if doSendGps else "Stopped") + " Sending GPS Data")              
                    elif inString[0:3] is "alt":
                        doSendAlt = not doSendAlt
                        radio.sendString(("Now" if doSendAlt else "Stopped") + " Sending Altimeter Data")              
                    elif inString[0:3] is "imu":
                        doSendImu = not doSendImu
                        radio.sendString(("Now" if doSendImu else "Stopped") + " Sending IMU Data")              
                    elif inString[0:3] is "mag":
                        doSendMag = not doSendMag
                        radio.sendString(("Now" if doSendMag else "Stopped") + " Sending Magnometer Data")              
                    elif inString[0:3] is "pow":
                        doSendPow = not doSendPow
                        radio.sendString(("Now" if doSendPow else "Stopped") + " Sending Power Data")              
                    else:
                       radio.sendError("Command Not Understood")
                elif inString[0:3] is "led":
                    inString = inString[3:]
                    if inString[0:3] is "gps":
                        gpsLED.toggle()
                    elif inString[0:2] is "tx":
                        transmitLED.toggle()
                    elif inString[0:2] is "rx":
                        receiveLED.toggle()
                    elif inString[0:2] is "sen":
                        processLED.toggle()
                    elif inString[0:2] is "err":
                        errorLED.toggle()
                    else:
                       radio.sendError("Command Not Understood")
                else:
                    radio.sendError("Command Not Understood")
            elif inString[0:3] is "get":
                inString = inString[3:]
                if inString[0:6] is "dosend":
                    inString = inString[6:]
                    if inString[0:3] is "gps":
                        radio.sendString(str(doSendGps))
                    elif inString[0:3] is "alt":
                        radio.sendString(str(doSendAlt))
                    elif inString[0:3] is "imu":
                        radio.sendString(str(doSendImu))
                    elif inString[0:3] is "mag":
                        radio.sendString(str(doSendMag))
                    elif inString[0:3] is "pow":
                        radio.sendString(str(doSendPow))
                    else:
                        radio.sendError("Command Not Understood")
                elif inString[0:4] is "cube":
                    inString = inString[4:]
                    if inString[0:6] is "status":
                        radio.sendString("Alive")
                else:
                    radio.sendError("Command Not Understood")
            elif inString[0:5] is "reset":
                radio.sendError("Reseting...")
                microcontroller.reset()
            elif inString[0:9] is "runlights":
                startupLightshow()
            else:
                radio.sendError("Command Not Understood")
            
        except:
            radio.sendError("Failed To Interpret Command")        
        
    def sendData():
        if doSendGps:
            radio.sendString(gps.getData())
        if doSendAlt:
            radio.sendString(altimeter.getData())
        if doSendImu:
            radio.sendString(imu.getData())
        if doSendMag:
            radio.sendString(magnometer.getData())
        if doSendPow:
            radio.sendString(power.getData())
            
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
    gps = GPS()
    altimeter = Altimeter()
    imu = IMU()
    magnometer = Magnometer()
    power = Power()
    startupLightshow()

    radio.sendString("Cubesat Initialized")

    while True:
        processCommand()
        sendData()
        clock.sleep(2)
        processLED.toggle()
        receiveLED.turnOff()
    
except:
    radio.sendError("Critical Error Occured")
    print("Critical Error Occured")