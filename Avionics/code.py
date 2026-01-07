import board
import busio
import pwmio
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
    
    I2C0 = busio.I2C(board.GP1,board.GP0,frequency=10000)
    UART0 = busio.UART(board.GP16,board.GP17,baudrate=9600,timeout=10)

    class onboardDevice:
        def __init__(self):
            self.isFunctional = None
            self.boardArgs = []
        
        def getBoardArgs(self):
            rString = ""
            for i in range(0, len(self.boardArgs)):
                rString += self.boardArgs + "\n"
            return rString

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
            global sd
            radio.sendString("Loading Config")

            if not sd.isFunctional:
                radio.sendString("Failed To Load Config, SD Not Functional")
                return

            try:
                with open(sd.configPath, "r") as configFile:
                    newConfig = configFile.readlines()

                    if len(newConfig) == 0: 
                        error("Config File Is Empty")
                    else:
                        self.doSendGps = (True if newConfig[0] is "t" else False)
                        self.doSendAlt = (True if newConfig[1] is "t" else False)
                        self.doSendImu = (True if newConfig[2] is "t" else False)
                        self.doSendMag = (True if newConfig[3] is "t" else False)
                        self.doSendPow = (True if newConfig[4] is "t" else False)
                        self.doPing = (True if newConfig[5] is "t" else False)
                        #self.pingInterval = int(newConfig[6])
                        radio.sendString("Config Loaded")
            except:
                error("Failed To Load Configuration")
        
        def saveConfig(self):
            
            if not sd.isFunctional:
                radio.sendString("Failed To Save Config, SD Not Functional")
                return
            
            try:
                with open(sd.configPath, "w") as configFile:
                    configFile.write(
                        ("t"if self.doSendGps else"f")+"\n"+
                        ("t"if self.doSendAlt else"f")+"\n"+
                        ("t"if self.doSendImu else"f")+"\n"+
                        ("t"if self.doSendMag else"f")+"\n"+
                        ("t"if self.doSendPow else"f")+"\n"+
                        ("t"if self.doPing else"f")+"\n"+
                        str(self.pingInterval))

                radio.sendString("Config Updated")
            except:
                radio.sendString("Failed To Save Config")
        
    class SDCard(onboardDevice):
        def __init__(self):
            try:
                super().__init__()
                # Define interface
                self.interface = adafruit_sdcard.SDCard(SPI1,CS1)
                self.boardArgs = ["SD Card", SPI1, CS1]
                
                # Mount storage
                vfs = storage.VfsFat(self.interface)  
                storage.mount(vfs, "/sd")
                
                # Define file paths
                self.configPath = "/sd/config.txt"
                self.errorPath = "/sd/error.txt"
                
                # Open Files
                with open(self.configPath, "w") as file:
                    pass
                with open(self.errorPath, "w") as file:
                    pass

                self.writeToFile(self.configPath, "f\nf\nf\nf\nf\n1")
                self.isFunctional = True
            except:
                self.isFunctional = False
                radio.sendString("Failed To Initialize SD Card")

        def writeToFile(self, filePath, string):
            try:
                self.isFunctional = True

                with open(filePath, "w") as writeFile:
                    writeFile.write(string)
            except:
                self.isFunctional = False
                radio.sendString("Failed To Write To SD Card")
             
    class GPS(onboardDevice):
        def __init__(self):
            try:
                super().__init__()
                # Define access point for GPS Data
                self.interface = adafruit_gps.GPS(UART0,debug=False)
                self.boardArgs = ["GPS", UART0, "Debug: False"]
                
                #Issues parameters to GPS
                self.interface.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
                self.interface.send_command(b'PMTK220,1000')
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup GPS" )
        
        def getData(self):
            try:
                self.isFunctional = True
                
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
                self.isFunctional = False
                return "err Failed To Send GPS Data"

    class Altimeter(onboardDevice):
        def __init__(self):
            try:
                super().__init__()
                # Define access point for Altimeter Data
                self.interface = adafruit_bme680.Adafruit_BME680_I2C(I2C0, address=0x77)
                self.boardArgs = ["I2C", I2C0, "Address: 0x77"]

                # Set default parameters for the interface
                self.interface.sea_level_pressure = 1013.25
                self.interface.pressure_oversampling = 8
                self.interface.temperature_oversampling = 2
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup Altimeter ")
        
        def getData(self):
            try:
                self.isFunctional = True
                
                # Format Altimeter data as a string
                return "ALT {:.2f} {:.2f} {:.2f} {:.2f} {:.0f}".format(
                    self.interface.altitude,
                    self.interface.temperature,
                    self.interface.pressure,
                    self.interface.relative_humidity,
                    round(self.interface.gas))
            except:
                self.isFunctional = False
                return "err Failed To Send Altimeter Data"
                
    class IMU(onboardDevice):
        def __init__(self):
            try:
                super().__init__()
                # Define access point for IMU Data
                self.interface = LSM6DSOX(I2C0,address=0x6a)
                self.boardArgs = [I2C0, "Address: 0x6a"]
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup IMU ")
        def getData(self):
            try:
                self.isFunctional = True
                
                # Format IMU data as a string
                return "IMU {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}".format(
                    self.interface.acceleration[0],
                    self.interface.acceleration[1],
                    self.interface.acceleration[2],                
                    self.interface.gyro[0],
                    self.interface.gyro[1],
                    self.interface.gyro[2])
            except:
                self.isFunctional = False
                return "err Failed To Send IMU Data"
                
    class Magnometer(onboardDevice):
        def __init__(self):
            try:
                super().__init__()
                # Define access point for Magnometer Data
                self.interface = LIS3MDL(I2C0,address=0x1c)
                self.boardArgs = [I2C0, "Address: 0x1c"]
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup Magnometer ")
        def getData(self):
            try:
                self.isFunctional = True
                
                # Format Magnometer data as a string
                return "MAG {:.2f} {:.2f} {:.2f}".format(
                    self.interface.magnetic[0],
                    self.interface.magnetic[1],
                    self.interface.magnetic[2])              
            except:
                self.isFunctional = False
                return "err Failed To Send Magnometer Data"
                
    class Power(onboardDevice):
        def __init__(self):
            try:
                super().__init__()
                # Define access point for Power Draw Data
                self.powerDrawInterface = INA219(I2C0,addr=0x45)
                self.boardArgs = [I2C0, "Address: 0x45"]

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
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup Power ")
        def getData(self):
            try:
                self.isFunctional = True
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
                self.isFunctional = False
                return "err Failed To Send Power Data"

    class Solar(onboardDevice):
        def __init__(self, address):
            try:
                super().__init__()
                # Define access point for Solar Data
                self.interface = INA219(I2C0,addr=address)
                self.boardArgs = [I2C0, "Address: "+address]
                
                # Set default parameters for the interface
                self.interface.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.interface.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                self.interface.bus_voltage_range = BusVoltageRange.RANGE_16V
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup Solar ")
    
    class LED(onboardDevice):
        def __init__(self, boardPin):
            try:
                super().__init__()
                # Define access point LED Object
                self.boardPin = boardPin
                self.interface = digitalio.DigitalInOut(self.boardPin)
                self.interface.direction = digitalio.Direction.OUTPUT
                self.boardArgs = ["Digital IO", self.boardPin, "Direction: Out"]
                self.value = False;
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup LED ")
    
        def turnOn(self):
            try:
                self.isFunctional = True
                self.value = True
                self.interface.value = True
            except:
                self.isFunctional = False
                error("Failed To Turn On LED ")
        
        def turnOff(self):
            try:
                self.isFunctional = True
                self.value = False
                self.interface.value = False
            except:
                self.isFunctional = False
                error("Failed To Turn Off LED ")

        def toggle(self):
            try:
                self.isFunctional = True
                self.value = (not self.value)
                self.interface.value = self.value
            except:
                self.isFunctional = False
                error("Failed To Toggle LED ")
        
    class Tranciever(onboardDevice):
        def __init__(self):
            try:
                super().__init__()
                # Establish output Pins
                M0 = digitalio.DigitalInOut(board.GP6)
                M1 = digitalio.DigitalInOut(board.GP7)
                
                M0.direction = digitalio.Direction.OUTPUT
                M1.direction = digitalio.Direction.OUTPUT
                
                M0.value = False
                M1.value = False
                
                # Define access point for Tansciever
                self.interface = busio.UART(board.GP8,board.GP9,baudrate=9600,timeout=0.1)
                
                self.boardArgs = ["UART", board.GP8, board.GP9, "Baudrate: 9600", "Timeout: 0.1"]
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup Tranciever ")

        def sendString(self, string):
            try:
                self.isFunctional = True
                self.interface.write((string+"end_msg").encode("ascii"))
            except:
                self.isFunctional = False
                error("Failed To Send String ")
        def sendError(self, string):
            try:
                self.isFunctional = True
                self.interface.write(("err "+string+"end_msg").encode("ascii"))
            except:
                self.isFunctional = False
                error("Failed To Send Error ")
        def sendBytes(self, bytes):
            try:
                self.isFunctional = True
                self.interface.write(bytes)
            except:
                self.isFunctional = False
                error("Failed To Send Bytes ")
        def readIncoming(self):
            try:
                self.isFunctional = True
                return self.interface.read(960)
            except:
                self.isFunctional = False
                error("Failed To Read Incoming Data ")
    
    def error(errorMessage):
        sd.writeToFile(sd.errorPath, errorMessage)
        radio.sendError(errorMessage)
        errorLED.turnOn()
    
    def processCommand():
        # read the recieved data from the radio
        inString = str(radio.readIncoming())
        
        # if there is a command, toggle LED and continue
        if inString is not "None": receiveLED.turnOn()
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
                       error("Command Not Understood")
                       
                elif inString[0:7] is "pingint":
                    config.pingInterval = int(inString[7:])
                    radio.sendString("Ping Interval Now: " + str(config.pingInterval) + " Clock Cycles")
                    config.saveConfig()
                else:
                    error("Command Not Understood")
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
                    error("Command Not Understood")

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
                        error("Command Not Understood")
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
                    error("Command Not Understood")
            
            # Reset Cube
            elif inString[0:5] is "reset":
                #config.saveConfig()
                radio.sendString("Reseting...")
                microcontroller.reset()
            
            # Toggle lightshow
            elif inString[0:9] is "runlights":
                startupLightshow()
            else:
                error("Command Not Understood")
            
        except:
            error("Failed To Interpret Command")        
        
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
        if config.doPing and pingTimer >= config.pingInterval:
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
    #power = Power()
    sd = SDCard()
    # Load Data From Config
    config = cubesatConfig()
    config.loadConfig()
    
    clockTimer = 1
    pingTimer = 0

    # Visual startup
    startupLightshow()
    radio.sendString("Cubesat Initialized")

    while True:
        if (clock.monotonic()%clockTimer) == 0:
            processCommand() # Process incoming commands
            sendData() # Send any data that is toggled to be sent
            processLED.toggle() # Visualize clock cycle
            receiveLED.turnOff() # Reset recieve LED
            errorLED.turnOff() # Reset error LED
            pingTimer += 1 # Incriment Ping Timer
    
except:
    error("Critical Error Occured")