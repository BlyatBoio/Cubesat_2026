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
    # Define bord access points
    SPI1 = busio.SPI(board.GP10,MOSI=board.GP11,MISO=board.GP12)
    CS1 = digitalio.DigitalInOut(board.GP13)

    sdcard = adafruit_sdcard.SDCard(SPI1,CS1)
    vfs = storage.VfsFat(sdcard)
    storage.mount(vfs, "/sd")
    config_file = "config.txt"
        
    I2C0 = busio.I2C(board.GP1,board.GP0,frequency=10000)
    UART0 = busio.UART(board.GP16,board.GP17,baudrate=9600,timeout=10)

    class cubesatConfig:
        def __init__(self):
            self.doSendGps = False
            self.doSendAlt = False
            self.doSendImu = False
            self.doSendMag = False
            self.doSendPow = False
            self.doPing = False
            self.pingInterval = 1
        def loadConfig(self):
            with open("/sd/Config.txt", "r") as configFile:
                newConfig = configFile.readLines()

                if(newConfig[0] is ""): return

                self.doSendGps = True if newConfig[0]=="t" else False
                self.doSendAlt = True if newConfig[1]=="t" else False
                self.doSendImu = True if newConfig[2]=="t" else False
                self.doSendMag = True if newConfig[3]=="t" else False
                self.doSendPow = True if newConfig[4]=="t" else False
                self.doPing = True if newConfig[5]=="t" else False
                self.pingInterval = int(newConfig[6])
        
        def saveConfig(self):
            with open("/sd/Config.txt", "w") as configFile:
                configFile.write(
                    ("t"if self.doSendGps else"f")+"\n",
                    ("t"if self.doSendAlt else"f")+"\n",
                    ("t"if self.doSendImu else"f")+"\n",
                    ("t"if self.doSendMag else"f")+"\n",
                    ("t"if self.doSendPow else"f")+"\n",
                    ("t"if self.doPing else"f")+"\n",
                    str(self.pingInterval))

            radio.sendString("Config Updated")

     # Load Data From Config
    config = cubesatConfig()

    class GPS:
        def __init__(self):
            try:
                # Define access point for GPS Data
                self.interface = adafruit_gps.GPS(UART0,debug=False)
                
                #Issues parameters to GPS
                self.interface.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
                self.interface.send_command(b'PMTK220,1000')
            except:
                radio.sendError("Failed To Setup GPS" )
        
        def getData(self):
            try:
                # Collect data from GPS into interface object
                self.interface.update()

                # Format GPS Data as a string
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
                # Define access point for Altimeter Data
                self.interface = adafruit_bme680.Adafruit_BME680_I2C(I2C0, address=0x77)

                # Set default parameters for the interface
                self.interface.sea_level_pressure = 1013.25
                self.interface.pressure_oversampling = 8
                self.interface.temperature_oversampling = 2
            except:
                radio.sendError("Failed To Setup Altimeter ")
        
        def getData(self):
            # Format Altimeter data as a string
            return "ALT {:.2f} {:.2f} {:.2f} {:.2f} {:.0f}".format(
                self.interface.altitude,
                self.interface.temperature,
                self.interface.pressure,
                self.interface.relative_humidity,
                round(self.interface.gas))
            
    class IMU:
        def __init__(self):
            try:
                # Define access point for IMU Data
                self.interface = LSM6DSOX(I2C0,address=0x6a)
            except:
                radio.sendError("Failed To Setup IMU ")
        def getData(self):
            # Format IMU data as a string
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
                # Define access point for Magnometer Data
                self.interface = LIS3MDL(I2C0,address=0x1c)
            except:
                radio.sendError("Failed To Setup Magnometer ")
        def getData(self):
            # Format Magnometer data as a string
            return "MAG {:.2f} {:.2f} {:.2f}".format(
                self.interface.magnetic[0],
                self.interface.magnetic[1],
                self.interface.magnetic[2])              
                
    class Power:
        def __init__(self):
            try:
                # Define access point for Power Draw Data
                self.powerDrawInterface = INA219(I2C0,addr=0x45)

                # Set default parameters for the interface
                self.powerDrawInterface.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.powerDrawInterface.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.powerDrawInterface.bus_voltage_range = BusVoltageRange.RANGE_16V
                
                # Define access point for Solar Data
                self.solar1Inteface = Solar(0x40)
                self.solar2Inteface = Solar(0x41)
                self.solar3Inteface = Solar(0x44)
                
                # Define access point for Batery Data
                self.batteryInterface = adafruit_max1704x.MAX17048(I2C0,address=0x36)
            except:
                radio.sendError("Failed To Setup Power ")
        def getData(self):
            try:
                # Format Power Data (Currently without solar)
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
                return "err Failed To Send Power Data"

    class Solar:
        def __init__(self, adress):
            try:
                # Define access point for Solar Data
                self.interface = INA219(I2C0,addr=adress)
                
                # Set default parameters for the interface
                self.interface.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.interface.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.interface.bus_voltage_range = BusVoltageRange.RANGE_16V
            except:
                radio.sendError("Failed To Setup Solar ")
    
    class LED:
        def __init__(self, boardPin):
                # Define access point LED Object
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
            
            # Define access point for Tansciever
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
        # read the recieved data from the radio
        inString = str(radio.readIncoming())
        
        # if there is a command, toggle LED and continue
        if(inString is not "None"): receiveLED.turnOn()
        else: return
        
        # Format string to be more default and readable
        inString = inString.lower()
        inString = inString[2:] # remove(b')
        inString = inString.replace(" ", "")
        inString = inString.replace("'", "") # remove end '

        try:
            # Ping the cube for a response
            if inString[0:4] is "ping":
                radio.sendString("pong")
            elif inString[0:3] is "set":
                inString = inString[3:]

                # Toggle what data is sent down
                if inString[0:6] is "dosend":
                    inString = inString[6:]
                    value = True if inString[3:7] == "true" else False

                    if inString[0:3] is "gps":
                        config.doSendGps = value
                        radio.sendString(("Now" if config.doSendGps else "Stopped") + " Sending GPS Data")              
                        config.saveConfig()
                    elif inString[0:3] is "alt":
                        config.doSendAlt = value
                        radio.sendString(("Now" if config.doSendAlt else "Stopped") + " Sending Altimeter Data")              
                        config.saveConfig()
                    elif inString[0:3] is "imu":
                        config.doSendImu = value
                        radio.sendString(("Now" if config.doSendImu else "Stopped") + " Sending IMU Data")              
                        config.saveConfig()
                    elif inString[0:3] is "mag":
                        config.doSendMag = value
                        radio.sendString(("Now" if config.doSendMag else "Stopped") + " Sending Magnometer Data")              
                        config.saveConfig()
                    elif inString[0:3] is "pow":
                        config.doSendPow = value
                        radio.sendString(("Now" if config.doSendPow else "Stopped") + " Sending Power Data")              
                        config.saveConfig()
                    elif inString[0:3] is "pin":
                        config.doPing = value
                        radio.sendString(("Now" if config.doPing else "Stopped") + " Sending Ping On The Interval: "+str(config.pingInterval)+" Clock Cycles")              
                        config.saveConfig()
                    else:
                       radio.sendError("Command Not Understood")
                       
                elif inString[0:4] is "ping":
                    config.pingInterval = int(inString[4:])
                    radio.sendString("Ping Interval Now: " + str(config.pingInterval) + " Clock Cycles")
                    config.saveConfig()
                else:
                    radio.sendError("Command Not Understood")
            # Toggle on or off an LED
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

            # get different data
            elif inString[0:3] is "get":
                inString = inString[3:]

                # Get whether or not a data type is being sent down
                if inString[0:6] is "dosend":
                    inString = inString[6:]
                    if inString[0:3] is "gps":
                        radio.sendString(str(config.doSendGps))
                    elif inString[0:3] is "alt":
                        radio.sendString(str(config.doSendAlt))
                    elif inString[0:3] is "imu":
                        radio.sendString(str(config.doSendImu))
                    elif inString[0:3] is "mag":
                        radio.sendString(str(config.doSendMag))
                    elif inString[0:3] is "pow":
                        radio.sendString(str(config.doSendPow))
                    else:
                        radio.sendError("Command Not Understood")
                elif inString[0:6] is "config":
                    inString = inString[6:]
                    radio.sendString(
                        "\nSend GPS:"+str(config.doSendGps)+"\n"+
                        "Send Alt:"+str(config.doSendAlt)+"\n"+
                        "Send IMU:"+str(config.doSendImu)+"\n"+
                        "Send Mag:"+str(config.doSendMag)+"\n"+
                        "Send Pow:"+str(config.doSendPow)+"\n"+
                        "Send Ping:"+str(config.doPing)+"\n"+
                        "Ping Interval:"+str(config.pingInterval)+"\n")
                # Similar to ping but funner to type
                elif inString[0:4] is "cube":
                    inString = inString[4:]
                    if inString[0:6] is "status":
                        radio.sendString("Alive")
                else:
                    radio.sendError("Command Not Understood")
            
            # Reset Cube
            elif inString[0:5] is "reset":
                #config.saveConfig()
                radio.sendError("Reseting...")
                microcontroller.reset()
            
            # Toggle lightshow
            elif inString[0:9] is "runlights":
                startupLightshow()
            else:
                radio.sendError("Command Not Understood")
            
        except:
            radio.sendError("Failed To Interpret Command")        
        
    def sendData():
        # Check which types of data should be send down
        if config.doSendGps:
            radio.sendString(gps.getData())
        if config.doSendAlt:
            radio.sendString(altimeter.getData())
        if config.doSendImu:
            radio.sendString(imu.getData())
        if config.doSendMag:
            radio.sendString(magnometer.getData())
        if config.doSendPow:
            radio.sendString(power.getData())
        if(config.doPing and pingTimer > config.pingInterval):
            radio.sendString("Ping")
            pingTimer = 0
            
    def startupLightshow():
        # All on in sequence then all off in sequence
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
    
    # Define all LEDs
    imbeddedLED = LED(board.GP25)
    processLED = LED(board.GP21)    
    gpsLED = LED(board.GP22)
    transmitLED = LED(board.GP20)
    receiveLED = LED(board.GP18)
    errorLED = LED(board.GP19)

    # Define and initialize all onboard devices
    radio = Tranciever()
    gps = GPS()
    altimeter = Altimeter()
    imu = IMU()
    magnometer = Magnometer()
    power = Power()

    pingTimer = 0

    # Visual startup
    startupLightshow()
    radio.sendString("Cubesat Initialized")

    while True:
        processCommand() # Process incoming commands
        sendData() # Send any data that is toggled to be sent
        clock.sleep(2)
        processLED.toggle() # Visualize clock cycle
        receiveLED.turnOff() # Reset recieve LED
        pingTimer += 1 # Incriment Ping Timer
    
except:
    radio.sendError("Critical Error Occured")
    print("Critical Error Occured")