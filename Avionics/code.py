import board
import busio
import pwmio
import math
import random
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
    SPI1 = busio.SPI(board.GP10,MOSI=board.GP11,MISO=board.GP12) # SD Card
    CS1 = digitalio.DigitalInOut(board.GP13) # SD Card Chip Select
    
    PWMPin1 = board.GP27
    PWMPin2 = board.GP28

    I2C0 = busio.I2C(board.GP1,board.GP0,frequency=10000) # Altimeter, IMU, Magnometer, Power, Solar, Battery
    UART0 = busio.UART(board.GP16,board.GP17,baudrate=9600,timeout=10) # GPS
    
    class onboardDevice:
        """Simple parent class handling onboard devices"""
        def __init__(self):
            self.isFunctional = None # Set manually case by case in each class
            self.boardArgs = []
        
        def getBoardArgs(self) -> str:
            """Get a string representation of the arguments passed to the device
                Returns:
                    (str): a string definition of the device
            """
            rString = ""
            for i in range(0, len(self.boardArgs)):
                rString += self.boardArgs[i] + "\n"
            return rString

    class cubesatConfig:
        """Class To Store, Load and Save Config Data"""
        def __init__(self):
            # Define default config values
            self.doSendGps = False
            self.doSendAlt = False
            self.doSendImu = False
            self.doSendMag = False
            self.doSendPow = False
            self.doPing = False
            self.pingInterval = 1
            
        def loadConfig(self):
            """Load Config From SD Card"""
    
            global sd
            radio.sendString("Loading Config")

            if not sd.isFunctional:
                radio.sendString("Failed To Load Config, SD Not Functional")
                return

            try:
                with open(sd.configPath, "r") as configFile:
                    newConfig = configFile.readlines()

                    # Each line is an index up,
                    # Each line has either t or f representing true or fase
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
            """Save Config To SD Card"""
            
            if not sd.isFunctional:
                radio.sendString("Failed To Save Config, SD Not Functional")
                return
            
            # Sepparate each line by \n so .readLines can sepparate each value easily
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
        """Interface class for the SD Card"""
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
                
                # Open / create files if they dont exist
                with open(self.configPath, "w") as file:
                    pass
                with open(self.errorPath, "w") as file:
                    pass

                # Write default config to config file
                self.writeToFile(self.configPath, "f\nf\nf\nf\nf\n1")
                self.isFunctional = True
            except:
                self.isFunctional = False
                radio.sendString("Failed To Initialize SD Card")

        def writeToFile(self, filePath:str, string:str, writeType="w"):
            """Write a string to a file on the SD Card
                Args:
                    filePath (str): Path to file on SD Card
                    string (str): string to write to file
                    writeType (str): Whether to write, append, or create a file (w, a, x)
            """
            try:
                self.isFunctional = True # If it cant write a file, this value will be reset to false

                with open(filePath, writeType) as writeFile:
                    writeFile.write(string)
            except:
                self.isFunctional = False
                radio.sendString("Failed To Write To SD Card")
             
    class GPS(onboardDevice):
        """Interface class for the GPS"""
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
        
        def getData(self) -> str:
            """Get data from the GPS formatted for the groundstation
                Returns:
                    (str): Data string formatted as GPS {value1} {value2}...
            """
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
        """Interface class for the Altimeter"""
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
        
        def getData(self) -> str:
            """Get data from the Altimeter formatted for the groundstation
                Returns:
                    (str): Data string formatted as ALT {value1} {value2}...
            """
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
        """Interface class for the IMU"""
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

        def getData(self) -> str:
            """Get data from the IMU formatted for the groundstation
                Returns:
                    (str): Data string formatted as IMU {value1} {value2}...
            """
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
        """Interface class for the Magnometer"""
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
        
        def getData(self) -> str:
            """Get data from the Magnometer formatted for the groundstation
                Returns:
                    (str): Data string formatted as MAG {value1} {value2}...
            """
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
        """Interface class for the Power"""
        def __init__(self):
            try:
                super().__init__()
                # Define access point for Power Draw Data
                # self.powerDrawInterface = INA219(I2C0,addr=0x45)
                # self.boardArgs = [I2C0, "Address: 0x45"]

                # Set default parameters for the interface
                # self.powerDrawInterface.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                # self.powerDrawInterface.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
                # self.powerDrawInterface.bus_voltage_range = BusVoltageRange.RANGE_16V
                
                # Define access point for Solar Data
                # self.solar1Inteface = Solar(0x40)
                # self.solar2Inteface = Solar(0x41)
                # self.solar3Inteface = Solar(0x44)
                
                # Define access point for Batery Data
                # self.batteryInterface = adafruit_max1704x.MAX17048(I2C0,address=0x36)
                self.isFunctional = False
            except:
                self.isFunctional = False
                error("Failed To Setup Power ")
        
        def getData(self) -> str:
            """Get data from the Power formatted for the groundstation
                Returns:
                    (str): Data string formatted as POW {value1} {value2}...
            """
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
        """Interface class for the Solar"""
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
        """Interface class for the LEDS"""
        def __init__(self, boardPin):
            try:
                super().__init__()
                # Define access point LED Object
                self.boardPin = boardPin
                self.interface = digitalio.DigitalInOut(self.boardPin)
                self.interface.direction = digitalio.Direction.OUTPUT
                
                self.boardArgs = ["Digital IO", self.boardPin, "Direction: Out"]
                
                self.value = False
                self.isFunctional = True
            except:
                self.isFunctional = False
                error("Failed To Setup LED ")
    
        def turnOn(self):
            """Turn On the LED"""
            try:
                self.isFunctional = True
                self.value = True
                self.interface.value = True
            except:
                self.isFunctional = False
                error("Failed To Turn On LED ")
        
        def turnOff(self):
            """Turn Off the LED"""
            try:
                self.isFunctional = True
                self.value = False
                self.interface.value = False
            except:
                self.isFunctional = False
                error("Failed To Turn Off LED ")

        def toggle(self):
            """Toggle the LED"""
            try:
                self.isFunctional = True
                self.value = (not self.value)
                self.interface.value = self.value
            except:
                self.isFunctional = False
                error("Failed To Toggle LED ")
        
    class Tranciever(onboardDevice):
        """Interface class for the Tranciever"""
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

        def sendString(self, string:str):
            """Send a string via the Tranciever
                Args:
                    string (str): The string sent via the radio
            """
            try:
                self.isFunctional = True
                self.interface.write((string+"end_msg").encode("ascii"))
            except:
                self.isFunctional = False
                error("Failed To Send String")
                
        def sendError(self, string:str):
            """Send an error via the Tranciever
                Args:
                    string (str): The error sent via the radio
            """
            try:
                self.isFunctional = True
                self.interface.write(("err "+string+"end_msg").encode("ascii"))
            except:
                self.isFunctional = False
                error("Failed To Send Error ")
        
        def sendBytes(self, bytes:bytearray):
            """Send bytes via the Tranciever
                Args:
                    bytes (bytearray): The bytes send via the radio
            """
            try:
                self.isFunctional = True
                self.interface.write(bytes)
            except:
                self.isFunctional = False
                error("Failed To Send Bytes")
                
        def readIncoming(self) -> str:
            """Read incoming data from the Tranciever
                Returns:
                    (str): The data actively being read in from the tranciever
            """
            try:
                self.isFunctional = True
                return self.interface.read(960)
            except:
                self.isFunctional = False
                error("Failed To Read Incoming Data")

    LAST = 1
    RACE = 2
    class Command:
        """Class to handle scheduled execution of an action"""
        def __init__(self, action=lambda:Commands.doNothing(), requirements=[], finishConditions=[lambda:True]):
            """Initialize new Command
                Args:
                    action (callable): The lambda action to perform when the command executes
                    requirements([callable]): The required boolean callables that are checked before executing the command
                    finishConditions([callable]): The required boolean callables that will stop the command, by default ends after 1 execution
            """
            self.action = action #Lambda statement to be called in execute
            self.cmdID = Commands.numCommands # ID used to determine if a command is the same
    
            Commands.numCommands += 1
    
            # Scheduling booleans
            self.isFinished = False
            self.hasStarted = False

            # Lambda functions checked every execution to ensure the command will function
            if not isinstance(requirements, list): self.requirements = [requirements]
            else: self.requirements = requirements 
            
            # Lambda functions checked every execution to determine whether the command is finished or not
            if not isinstance(finishConditions, list): self.finishConditions = [finishConditions] 
            else: self.finishConditions = finishConditions
        
        def __eq__(self, other): 
            # Does this command equal another command
            return self.cmdID == other.cmdID
        
        def addRequirement(self, requirement:callable):
            """Add a requirement to be checked before the command is executed
                Args:
                    requirement (callable): The boolean callable that will be called and checked before running the command
            """
            if isinstance(requirement, list): self.requirements.extend(requirement) # Append a list
            else: self.requirements.append(requirement)

        def addFinishCondition(self, condition:callable):
            """Add a condition that will prevent the command from being finished until it returns true
                Args:
                    condition (callable): The boolean callable that will end the command upon returning true
            """
            if isinstance(condition, list): self.finishConditions.extend(condition) # Append a list
            else: self.finishConditions.append(condition)

        def checkRequirements(self) -> bool:
            """Check every requirement and return if they are all met
                Returns:
                    (bool): whether or not all of the requirements are met
            """
            # if there are no requirements, return true
            if len(self.requirements) == 0: 
                return True
            
            # Check all requirements
            for requirement in self.requirements:
                if (not requirement()):
                    return False
            return True

        def checkFinishConditions(self) -> bool:
            """Check every finish condition and return if they are all met
                Returns:
                    (bool): whether or not all of the conditions are met
            """
            for condition in self.finishConditions:
                if not condition():
                    return False
            return True

        def start(self):
            """Start the command"""
            self.hasStarted = True
            self.isFinished = False
            Commands.runCommand(self)

        def execute(self):
            """Perform the action and handle requirements and finish conditions"""
            if self.checkRequirements():
                self.action()
                if self.checkFinishConditions(): self.cancel()
        
        def andThen(self, Command:Command) -> commandSequence:
            """Add a command to be run once this command finishes
                Args:
                    Command (Command): The command to be added after this command
                Returns:
                    (commandSequence): New command sequence with this command and the one added
            """
            if isinstance(self, commandSequence): self.addCommand(Command)
            else: self = commandSequence([self, Command])
            return self
        
        def raceWith(self, Command:Command) -> parallelCommandSequence:
            """Add a command to be run along side this command and end both commands when the first finishes
                Args:
                    Command (Command): The command to be raced with this command
                Returns:
                    (parallelCommandSequence): New parallel command sequence with this command and the one added
            """
            if isinstance(self, parallelCommandSequence): self.addCommand(Command)
            else: self = parallelCommandSequence([self, Command], [], RACE)
            return self
        
        def runWith(self, Command:Command) -> parallelCommandSequence:
            """Add a command to be run along side this command        
                Args:
                    Command (Command): The command to be run alongside with this command
                Returns:
                    (parallelCommandSequence): New parallel command sequence with this command and the one added
            """
            if isinstance(self, parallelCommandSequence): self.addCommand(Command)
            else: self = parallelCommandSequence([self, Command], [], LAST)
            return self
        
        def until(self, condition:callable) -> Command:
            """Add a finish condition
                Args:
                    condition (callable): The condition to be added
                Returns:
                    (Command): The same command with the new condition added
            """
            self.addFinishCondition(condition)
            return self

        def cancel(self):
            """Stop the command from running"""
            self.isFinished = True
            self.hasStarted = False
            Commands.removeCommand(self)

    class parallelCommandSequence(Command):
        """List of commands run in parallel"""
        def __init__(self, commands=[], requirements=[], finishCondition=LAST):
            """Initialize new parallel Command
                Args:
                    commands ([Command]): The Commands to be run in parallel
                    requirements([callable]): The required boolean callables that are checked before executing the command
                    finishCondition: the condition upon which the entire parallel comnmand sequence will end
            """
            # Define end condition
            finishLam = lambda: self.oneCommandFinished()
            if finishCondition == LAST: finishLam = lambda: self.allCommandsFinished() 
            
            super().__init__(lambda: Commands.doNothing(), requirements, [finishLam]) # Initialzie parent command class
            self.commands = commands

            # Ensure the total command will not start until all underlying requirements are met
            for cmd in commands:
                self.addRequirement(cmd.requirements)
        
        def addCommand(self, command:Command):
            """Add a command to be run in parallel
                Args:
                    command (Command): the command to be run in parallel
            """
            self.commands.append(command)
            self.addRequirement(command.requirements)
        
        def allCommandsFinished(self) -> bool:
            """Check if all the commands are finished / LAST finish condition
                Returns:
                    (bool): whether or not all commands have finished
            """
            for cmd in self.commands:
                if not cmd.isFinished:
                    return False
            return True
        
        """Check if one the command is finished / RACE finish condition"""
        def oneCommandFinished(self) -> bool:
            """Check if one the command is finished / RACE finish condition
                Returns:
                    (bool): whether or not one command has finished
            """
            for cmd in self.commands:
                if cmd.isFinished:
                    return True
            return False
        
        def start(self):
            """Override start in Command to also start its list of commands"""
            self.hasStarted = True
            self.isFinished = False
            Commands.runCommand(self)
            for cmd in self.commands:
                cmd.start()

    class commandSequence(Command):
        """Holds a list of commands that are run in sequence, functions also as a command itself"""
        
        def __init__(self, commands=[], requirements=[]):
            """Initialize new Command sequence
                Args:
                    commands ([Command]): The Commands to be run in parallel
                    requirements([callable]): The required boolean callables that are checked before executing the command
            """
            
            super().__init__(lambda: self.runCommands(), requirements, [lambda: self.getIsFinished()]) # initialize parent command class
            
            # Sequential variables
            self.commands = commands
            self.currentCommandIndex = 0

            # Ensure the total command will not start until all underlying requirements are met
            for cmd in commands:
                self.addRequirement(cmd.requirements)
        
        def runCommands(self):
            """Function called as the action of the parent command class"""
            if  self.currentCommandIndex == 0 or self.commands[self.currentCommandIndex-1].isFinished:
                self.commands[self.currentCommandIndex].start()
                self.currentCommandIndex += 1

        def getIsFinished(self) -> bool:
            """Function passed in as a lambda for the finishCondition of the parrent command class
                Returns:
                    (bool): Whether or not the sequence has reached the end of the list
            """
            return self.currentCommandIndex > len(self.commands)-1

        def addCommand(self, command:Command):
            """Add a command to the end of the list of commands
                Args:
                    command (Command): Command to be added to the list of commands
            """
            self.commands.append(command)
            self.addRequirement(command.requirements)

    class Commands:
        """Class for internal handling and running of commands"""
        runningCommands = []
        commandsToRun = []
        commandsToRemove = []
        numCommands = 0

        def update():
            """Run commands and update running command"""
            # Add commands added on the last frame
            if len(Commands.commandsToRun) > 0:
                Commands.runningCommands.extend(Commands.commandsToRun)
                Commands.commandsToRun.clear()

            # Remove commands removed on the last frame
            for cmd in Commands.commandsToRemove:

                newArray = []
                for cmd2 in Commands.runningCommands:
                    if cmd is not cmd2: newArray.append(cmd2)
                Commands.runningCommands = newArray.copy()
            
            Commands.commandsToRemove.clear()

            # Run all commands 
            for cmd in Commands.runningCommands:
                cmd.execute()                
        
        def runCommand(Command:Command):
            """Add command to be run on the next frame
                Args:
                    Command (Command): Command to be run on the next frame
            """
            Commands.commandsToRun.append(Command)
        
        def removeCommand(Command:Command):
            """Remove a command from the running commands on the next frame
                Args:
                    Command (Command): Command to be removed on the next frame
            """
            Commands.commandsToRemove.append(Command)
        
        def getWaitCommand(seconds:float) -> Command:
            """Get a command to wait a given period of time
                Args:
                    seconds (float): number of seconds to wait
                Returns:
                    (Command): Command to wait the given length of seconds
            """
            t = timer()
            return Command(lambda: Commands.doNothing(), [], [lambda: t.getElapsedTime() > seconds])

        def doNothing():
            """Get a command to pass into a lambda to do nothing"""
            pass

    class FlywheelMotor(onboardDevice):
        """Interface class for the Flywheel Motor"""
        def __init__(self):
            """Creates a new flywheel motor interface"""
            super().__init__()
            # ESC has a typical frequency of 50 hz 
            self.frequency = 50
            # ESC has a duty cycle range of 1100 -> 1900 us out of 50 hz (20000 us) mapped to an integer in the range 0 -> 2^16
            self.duty_cycle = int(1900/20000) * 65535

            # Define access point for Flywheel Motor
            # Variable frequency is also required to be true if you are changing duty cycle which we are
            self.interface = pwmio.PWMOut(PWMPin1, frequency=self.frequency, duty_cycle=self.duty_cycle, variable_frequency=True)
            
            # Define PID controler
            self.pid = PIDController(1, 0.1, 0.1)
            
            # Control variables
            self.currentPercentOutput = 0
            self.targetSetpoint = 0
            self.currentCommand = Command()

        def setFrequency(self, frequencyIn:int):
            """Set the output frequency of the PWM Pin of the Flywheel motor (hz)
                Args:
                    frequencyIn (int): The frequency in (hz) to set the output pin
            """
            frequencyIn = int(frequencyIn)
            self.frequency = frequencyIn
            self.interface.frequency = frequencyIn
            
        def setDutyCycle(self, dutyCycleIn:int):
            """Set the output duty cycle of the PWM Pin of the Flywheel motor (0->2^16)
                Args:
                    dutyCycleIn (int): An integer in the range (0 -> 2^16) to set the duty cycle of the output pin
            """
            dutyCycleIn = int(dutyCycleIn)
            self.duty_cycle = dutyCycleIn
            self.interface.duty_cycle = self.duty_cycle
            
        def updateControl(self):
            """Update PID Control"""
            self.currentPercentOutput += self.pid.getControlOutput(self.currentPercentOutput)
            
        def setTargetSetpointCommand(self, targetPercent:float) -> Command:
            """Get a command to ramp the velocity of the flywheel to a given setpoint via PID
                Args:
                    targetPercent (float): the percent to ramp the velocity to
                Returns:
                    (Command): Command that when started will ramp the velocity of the flywheel over time
            """ 
            
            # update target setpoints
            self.pid.setSetpoint(targetPercent)
            self.targetSetpoint = targetPercent
            
            # Cancel currently running target setpoint command if this is called in the middle of another target setpoint, then run the default pid control loop
            self.currentCommand = Command(lambda: self.currentCommand.cancel()).andThen(Command(lambda: self.updateControl(), [], [lambda: self.hasReached()]))
            return self.currentCommand

        def hasReached(self) -> bool:
            """Get whether or not the flywheel has reached its target velocity setpoint
                Returns:
                    (bool): Whether or not the flywheel has reached its setpoint
            """
            return (self.targetSetpoint - self.currentPercentOutput < 1)
        
        def setPowerPercent(self, percent:float):
            """Set the requested output of the flywheel to a given percentage
                Args:
                    percent (float): the percent (0-100)% power output to set the motor to
            """
            percent = max(0, min(100, percent)) # clamp from 0 -> 100%
            self.currentPercentOutput = percent
            
            pulse_width = 1100 + ((percent / 100) * 800) # convert from 0->100 to 1100 -> 1900 (us range of duty cycle)
            dutyCycle = int((pulse_width / 20000) * 65535) # convert from % of frequency to number between 0 and 2^16
            self.setDutyCycle(dutyCycle) # Set output duty cycle
            
        def initMotorCommand(self) -> Command:
            """Get a command that runs the initalize sequence of the flywheel motor
                Returns:
                    (Command): Command that intiazizes / "arms" motor
            """
            return Command(lambda: self.setPowerPercent(100)).andThen(Commands.getWaitCommand(4)).andThen(Command(lambda: self.setPowerPercent(0)))
        
    class DirectorMotor(onboardDevice):
        """Interface Class for the Servo Motor"""
        def __init__(self):
            super().__init__()
            # Servo has a typical frequency of 50 hz 
            self.frequency = 50
            # Servo has a duty cycle range of 500 -> 2500 us out of 50 hz (20000 us) mapped to an integer in the range 0 -> 2^16
            self.duty_cycle = int(500/20000) * 65535
            
            # Define access point for Flywheel Motor
            self.interface = pwmio.PWMOut(PWMPin2, frequency=self.frequency, duty_cycle=self.duty_cycle, variable_frequency=True) # Flywheel Servo Motor PWM
            self.boardArgs = ["PWM", PWMPin2, self.frequency, self.duty_cycle]
            
            # Control variables
            self.currentAngle = 0
            self.angleStep = 0.05

        def setFrequency(self, frequencyIn:int):
            """Set the output frequency of the PWM Pin of the Flywheel motor (hz)
                Args:
                    frequencyIn (int): The frequency in (hz) to set the output pin
            """
            dutyCycleIn = int(dutyCycleIn)
            self.frequency = frequencyIn
            self.interface.frequency = frequencyIn
            
        def setDutyCycle(self, dutyCycleIn:int):
            """Set the output duty cycle of the PWM Pin of the Servo motor (0->2^16)
                Args:
                    dutyCycleIn (int): An integer in the range (0 -> 2^16) to set the duty cycle of the output pin
            """
            dutyCycleIn = int(dutyCycleIn)
            self.duty_cycle = dutyCycleIn
            self.interface.duty_cycle = self.duty_cycle

        def setTargetPositionCommand(self, degrees:float) -> Command:
            """Get a command to step the position from its current angle to the target angle
                Args:
                    target (float): the angle (0-180) to step the servo to  
                Returns:
                    (Command): command that will step from current angle to the provided target angle
            """
            return Command(lambda: self.stepServoRotationTo(degrees), [], [lambda: self.hasReachedTarget(degrees)])
        
        def hasReachedTarget(self, target:float) -> bool:
            """Boolean conditional to check if the current angle out is within an acceptable error of the target value
                Args:
                    target (float): the angle (0-180) to set the motor to
                Returns:
                    (bool): whether or not the servo has reached its target percentage
            """
            return abs(self.currentAngle - target) < 1

        def stepServoRotationTo(self, target:float):
            """Step the angle of the servo by a pre-defined step size towards the provided target
                Args:
                    target (float): the angle (0-180) to set the servo to
            """
            if self.currentAngle < target: self.setServoRotationTo(self.currentAngle+self.angleStep)
            else: self.setServoRotationTo(self.currentAngle-self.angleStep)

        def setServoRotationTo(self, degrees:float):
            """Set the requested angle of the servo to a given angle
                Args:
                    target (float): the angle (0-180) to set the servo to
            """
            self.currentAngle = degrees
            
            # Constrain angle to 0-180
            degrees = max(0, min(180, degrees))
            
            # Map angle to pulse width (500 to 2500 us)
            pulse_width = 500 + ((degrees / 180) * 2000)
            
            # Calculate 16-bit duty cycle (0-65535)
            duty_cycle = int((pulse_width / 20000) * 65535)
            
            self.setDutyCycle(duty_cycle)
        
    class RotationalControlSystem():
        def __init__(self):
            self.targetRotation = 0
            self.currentDegreesRotated = 0
            self.pid = PIDController(1, 0.1, 0.1)
            self.currentCommand = Command
        
        def setTargetRotation(self, targetRotation:float):
            """Set the target rotation of the flywheel to a given angle
                Args:
                    percent (float): the angle to rotate
            """
            self.currentCommand.cancel()
            self.currentDegreesRotated = 0
            self.targetRotation = targetRotation
            self.pid.setSetpoint(targetRotation)
        
        def updateControl(self):
            """Update PID Control"""
            pidOut = self.pid.getControlOutput(self.currentDegreesRotated)
            flywheel.setTargetSetpointCommand(abs(pidOut)).start()
            director.setTargetPositionCommand(pidOut > 0 if 15 else 165) # if the velocity is positive or negative, place the servo in the appropriate spot
            self.currentDegreesRotated += imu.interface.gyro[2]
        
        def hasReached(self) -> bool:
            """Get whether or not the cube has rotated the target ammount
                Returns:
                    (bool): Whether or not the cube has rotated the target ammount
            """
            return abs(self.targetRotation - self.currentDegreesRotated) < 1
        
        def getRotationCommand(self, targetRotation:float) -> Command:
            """Get a command to rotate the cube a given amount in degrees
                Args:
                    targetRotation (float): Rotation in degrees to rotate the cube
            """
            self.currentCommand = Command(lambda: self.setTargetRotation(targetRotation)).andThen(Command(lambda: self.updateControl(), [], [lambda:self.hasReached()]))
            return self.currentCommand
    
    class PIDController:
        """Control class to take in a PID tuning and use closed loop feedback to provide a dynamic curve"""
        def __init__(self, Kp:float, Ki:float, Kd:float):
            """Create new PID Controller with tuning Kp, Ki, Kd
                Args:
                    Kp (float): P-term multiplier / how much error existing increases the speed it approaches the target
                    Ki (float): I-term multiplier / how much time passing increases the speed it approaches the target
                    Kd (float): D-term multiplier / how much the decrease in error over time decreases the speed it approaches the target
            """
            # Control Tuning Variables
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            
            # Control Variables
            self.accumulatedError = 0
            self.pastError = 0
            self.setpoint = 0
        
        def setSetpoint(self, value:float):
            """Set the setpoint value that the PID controller will attempt to reach
                Args:
                    value (float): The new value of the setpoint
            """
            self.setpoint = value
            # when target changes, the error will be different and thus is reset
            self.accumulatedError = 0
            self.pastError = 0
        
        def getControlOutput(self, measuredValue:float, timeStep=0.1) -> float:
            """Get the calculated rate of change of the value based on the provided measured value and past error data
                Args:
                    measuredValue (float): The measured value of the value the PID Controller is controling
                    timeStep (float): The time step since the last control output calculation
                Returns:
                    (float): The calculated rate of change to apply to the controled variable
            """
            err = self.getError(measuredValue) # get how far the measured value is from the target value
            
            # Kp: P-term multiplier / how much error existing increases the speed it approaches the target
            # Ki: I(ntegral)-term multiplier / how much time passing increases the speed it approaches the target
            # Kd: D(erivative)-term multiplier / how much the decrease in error over time decreases the speed it approaches the target
            
            # Calculate control output
            controlOutput = (self.Kp * err) + (self.Ki * self.accumulatedError * timeStep) + ((err - self.pastError) / timeStep)
            
            # update control variables
            self.accumulatedError += err
            self.pastError = err
            
            return controlOutput
        
        def getError(self, measuredValue:float) -> float:
            """Get how far the current measured variable is from the setpoint
                Args:
                    measuredValue (float): The measured value of the value the PID Controller is controling
                Returns:
                    (float): The current error / distance between the measured value and setpoint
            """
            return self.setpoint - measuredValue

    class timer:
        """Simple timer class"""
        def __init__(self):
            self.startTime = clock.monotonic()
        
        def reset(self):
            """Reset the timer to 0"""
            self.startTime = clock.monotonic()
        
        def getElapsedTime(self) -> float:
            """Get the elapsed time since the last reset
                Returns:
                    (float): The elapsed time in seconds
            """
            return clock.monotonic() - self.startTime
        
    def error(errorMessage:str):
        """Send an error via radio and log it to the SD Card
            Args:
                errorMessage (str): The error to be saved on the SD and sent via the radio
        """
        sd.writeToFile(sd.errorPath, errorMessage, "a")
        radio.sendError(errorMessage)
        errorLED.turnOn()
    
    def saveValue(label:str, value:str):
        """Save a value to the SD Card
            Args:
                lable (str): the lable of the data value saved
                value (str): the value to save with the given lable
        """
        try:
            sd.writeToFile(f"/sd/data/log_{numLogs}.txt", label + ": " + value + "\n", "a")
        except:
            error("SD Is Not Functional, Could Not Save Value")

    def saveAllData():
        """Collect all data from the sensors and save each value individually in a new log file"""
        if not sd.isFunctional: return

        global numLogs
        numLogs += 1
        
        # create / open new file if it does not already exist
        with open(f"/sd/data/log_{numLogs}.txt", "w") as logFile:
            pass
                
        gpsData = gps.getData().split(" ")
        if gpsData[0] != "err":
            saveValue("Device",gpsData[0])
            saveValue("Lat",gpsData[1])
            saveValue("Long",gpsData[2])
            saveValue("Alt",gpsData[3])
            saveValue("Speed",gpsData[4])
            saveValue("Sat",gpsData[5])
            saveValue("HD",gpsData[6])
        else:
            saveValue("Error", "Saving GPS Data")

        altData = altimeter.getData().split(" ")
        saveValue("Device",altData[0])
        saveValue("Alt",altData[1])
        saveValue("Temp",altData[2])
        saveValue("Pres",altData[3])
        saveValue("Hum",altData[4])
        saveValue("Gas",altData[5])

        imuData = imu.getData().split(" ")
        saveValue("Device",imuData[0])
        saveValue("Acc X",imuData[1])
        saveValue("Acc Y",imuData[2])
        saveValue("Acc Z",imuData[3])
        saveValue("Rot X",imuData[4])
        saveValue("Rot Y",imuData[5])
        saveValue("Rot z",imuData[6])
        
        magData = magnometer.getData().split(" ")
        saveValue("Device",magData[0])
        saveValue("Mag X",magData[1])
        saveValue("Mag Y",magData[2])
        saveValue("Mag Z",magData[3])
        
        powData = power.getData().split(" ")
        if powData[0] != "err":
            saveValue("Device",powData[0])
            saveValue("Volts",powData[1])
            saveValue("Current",powData[2])
            saveValue("Watts",powData[3])
            saveValue("Bat Percent",powData[4])
            saveValue("Bat Volts",powData[5])
        else:
            saveValue("Error", "Saving Power Data")

    def processCommand(inString:str):
        """Process an incoming command string
            Args:
                inString (str): the string to process
        """
        if inString is not "None" : receiveLED.turnOn()
        else: return 
         
        # Format string to be more default and processable
        inString = inString.lower()
        inString = inString[2:] # remove(b') from the start
        inString = inString.replace(" ", "") # remove spaces so a command might look like setdosendgpsfalse so we dont need to account for spaces
        inString = inString.replace("'", "") # remove end '

        try:
            # Ping the cube for a response
            if inString[0:4] is "ping":
                radio.sendString("pong")
            # Set data 
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
                    elif inString[0:3] is "all":
                        config.saveConfig()

                    else:
                       error("Command Not Understood")
                       
                elif inString[0:7] is "pingint":
                    config.pingInterval = int(inString[7:])
                    radio.sendString("Ping Interval Now: " + str(config.pingInterval) + " Clock Cycles")
                    config.saveConfig()
                
                elif inString[0:5] is "servo":
                    director.setTargetPositionCommand(float(inString[5:]))
                    
                elif inString[0:8] is "flywheel":
                    flywheel.setTargetSetpointCommand(float(inString[8:]))
                
                elif inString[0:8] is "rotation":
                    rotationControl.setTargetRotation(float(inString[8:]))
                    
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
                elif inString[0:4] is "data":
                    inString = inString[4:]
                    if inString[0:3] is "gps":
                        radio.sendString(str(gps.getData()))
                    elif inString[0:3] is "alt":
                        radio.sendString(str(altimeter.getData()))
                    elif inString[0:3] is "imu":
                        radio.sendString(str(imu.getData()))
                    elif inString[0:3] is "mag":
                        radio.sendString(str(magnometer.getData()))
                    elif inString[0:3] is "pow":
                        radio.sendString(str(power.getData()))
                    else:
                        error("Command Not Understood")
                # Similar to ping but funner to type
                elif inString[0:4] is "cube":
                    inString = inString[4:]
                    if inString[0:6] is "status":
                        radio.sendString("Alive")
                else:
                    error("Command Not Understood")
            # Save different data to SD Card            
            elif inString[0:4] is "save":
                inString = inString[4:]
                if inString[0:3] is "gps":
                    saveValue("GPS Save", str(gps.getData()))
                elif inString[0:3] is "alt":
                    saveValue("Altimeter Save", str(altimeter.getData()))
                elif inString[0:3] is "imu":
                    saveValue("IMU Save", str(imu.getData()))
                elif inString[0:3] is "mag":
                    saveValue("Magnometer Save", str(magnometer.getData()))
                elif inString[0:3] is "pow":
                    saveValue("Power Save", str(power.getData()))
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
            #Evaluate arbitrary code
            elif inString[0:4] is "eval":
                try:
                    eval(inString[4:])
                except:
                    error("Failed To Evaluate Command")
            #Initialize Flywheel motor
            elif inString[0:9] is "initmotor":
                flywheel.initMotorCommand().start()
            else:
                error("Command Not Understood")
            
        except:
            error("Failed To Interpret Command")  
       
    def sendData():
        """Send data based on config toggles""" 
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
        """Visual startup lightshow"""
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

    commandTimer = timer()

    # Define and initialize all onboard devices
    radio = Tranciever()
    gps = GPS()
    altimeter = Altimeter()
    imu = IMU()
    magnometer = Magnometer()
    power = Power()
    flywheel = FlywheelMotor()
    director = DirectorMotor()
    sd = SDCard()
    rotationControl = RotationalControlSystem()

    # LAoad Data From Config
    config = cubesatConfig()
    config.loadConfig()

    # Timing Intervals
    clockTimer = 2
    pingTimer = 1
    
    numLogs = 0
    
    # Visual startup
    startupLightshow()
    radio.sendString("Cubesat Initialized")
    
    while True:
        Commands.update()

        if abs(clock.monotonic()%clockTimer) == 0:
            processCommand(str(radio.readIncoming())) # Process incoming commands
            sendData() # Send any data that is toggled to be sent
            saveAllData() # Save all data to log files on SD
            processLED.toggle() # Visualize clock cycle
            receiveLED.turnOff() # Reset recieve LED
            errorLED.turnOff() # Reset error LED
            pingTimer += 1 # Incriment Ping Timer     

except:
    error("Critical Error Occured")