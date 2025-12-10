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

import random



try:
    
    ###################################################################
    #                                                                 #
    #                     CC BY-NC-SA 4.0 License                     #
    #                                                                 #
    #   Except where otherwise noted, this code is licensed under a   #
    # Attribution-NonCommercial-ShareAlike 4.0 International License. #
    #                                                                 #
    ###################################################################

    ###################################################################################################################

    callsign = "PiCarrier" #[CHANGE BEFORE LAUNCH] Sets the callsign for the device (Line one in Configuration.txt)
    beaconInterval = 5 #Controls the interval of beacon signal (Line two in Configuration.txt)

    measurementInterval = 10 #Controls interval of data collection (Line three in Configuration.txt)
    maxDataNumber = 100000 #Maximum number of data measurements stored on the SD card (Line four in Configuration.txt)

    maxErrorNumber = 100 #Maximum number of error messages stored on the SD card (Line five in Configuration.txt)

    ###################################################################################################################

    #Establishes data pins
    I2C0 = busio.I2C(board.GP1,
                     board.GP0,
                     frequency=10000)

    SPI1 = busio.SPI(board.GP10,
                     MOSI=board.GP11,
                     MISO=board.GP12)

    CS1 = digitalio.DigitalInOut(board.GP13)

    UART0 = busio.UART(board.GP16,
                       board.GP17,
                       baudrate=9600,
                       timeout=10)

    UART1 = busio.UART(board.GP8,
                       board.GP9,
                       baudrate=9600,
                       timeout=0.1)

    # Sets up analog pins (may read non-zero value when not connected due to noise)
    GPIO26 = analogio.AnalogIn(board.GP26)
    GPIO27 = analogio.AnalogIn(board.GP27)
    GPIO28 = analogio.AnalogIn(board.GP28)

    #Established LED Outputs
    imbeddedLed = digitalio.DigitalInOut(board.GP25)
    imbeddedLed.direction = digitalio.Direction.OUTPUT

    processLed = digitalio.DigitalInOut(board.GP21)
    processLed.direction = digitalio.Direction.OUTPUT

    gpsLed = digitalio.DigitalInOut(board.GP22)
    gpsLed.direction = digitalio.Direction.OUTPUT

    transmitLed = digitalio.DigitalInOut(board.GP20)
    transmitLed.direction = digitalio.Direction.OUTPUT

    receiveLed = digitalio.DigitalInOut(board.GP18)
    receiveLed.direction = digitalio.Direction.OUTPUT

    errorLed = digitalio.DigitalInOut(board.GP19)
    errorLed.direction = digitalio.Direction.OUTPUT

    #Established LoRa Outputs
    M0 = digitalio.DigitalInOut(board.GP6)
    M0.direction = digitalio.Direction.OUTPUT
    M0.value = False

    M1 = digitalio.DigitalInOut(board.GP7)
    M1.direction = digitalio.Direction.OUTPUT
    M1.value = False

    #Defines sensors globally
    transceiver = None
    gps = None
    bme = None
    imu = None
    mag = None
    powerDraw = None
    solarOne = None
    solarTwo = None
    solarThree = None
    battery = None
    adc = None

    #Defines constants for sensor calibration
    temperature_offset = 0 #Each temperature sensor can be different, calibrate to a know temperature if possible
    vbus_reference_voltage = 5 #4.5V when powered by battery, 5V when powered by USB
    
    battery_detection_threshold = 2.5 #Determines when the battery is connected or not

    gpsStatus = False #Stores status of GPS connection for the GPS LED

    #Defines SD and storage card globally and sets dataNumber
    sdcard = None
    dataNumber = -1 #Line one in StateBackup.txt

    #Defines the sequence of beacon data globally
    beaconSequence = 0 #Line two in StateBackup.txt

    #Defines the errorNumber counter globally
    errorNumber = -1 #Line three in StateBackup.txt

    #Defines GPS object
    class GPS: 
        
        #GPS constructor
        def __init__(self, Latitude, Longitude, Altitude, Speed, Satellites, DOP):
            
            self.Latitude = Latitude
            self.Longitude = Longitude
            self.Altitude = Altitude
            self.Speed = Speed
            self.Satellites = Satellites
            self.DOP = DOP
        
        #Defines how GPS will be printed
        def __repr__(self):
            
            try:
            
                return "<Lat: {:.6f} degrees, Long: {:.6f} degrees, Altitude: {:.2f} m, Speed: {:.2f} m/s, Satellites: {}, DOP: {}>".format(self.Latitude,
                                                                                                                                    self.Longitude,
                                                                                                                                    self.Altitude,
                                                                                                                                    self.Speed,
                                                                                                                                    self.Satellites,
                                                                                                                                    self.DOP)
            
            except:
                
                return "<Lat: {} degrees, Long: {} degrees, Altitude: {} m, Speed: {} m/s, Satellites: {}, DOP: {}>".format(self.Latitude,
                                                                                                                            self.Longitude,
                                                                                                                            self.Altitude,
                                                                                                                            self.Speed,
                                                                                                                            self.Satellites,
                                                                                                                            self.DOP)

    #Defines ALTIMETER object
    class ALTIMETER:
        
        #ALTIMETER constructor
        def __init__(self, Altitude, Temperature, Pressure, Humidity, Gas):
            
            self.Altitude = Altitude
            self.Temperature = Temperature
            self.Pressure = Pressure
            self.Humidity = Humidity
            self.Gas = Gas
        
        #Defines how ALTIMETER will be printed
        def __repr__(self):
            
            try:
                
                return "<Altitude: {:.2f} m, Temperature: {:.2f} C, Pressure: {:.2f} kPa, Humidity {:.2f} %, Gas: {:.0f} ohms>".format(self.Altitude,
                                                                                                                                       self.Temperature,
                                                                                                                                       self.Pressure,
                                                                                                                                       self.Humidity,
                                                                                                                                       self.Gas)
        
            except:
                
                return "<Altitude: {} m, Temperature: {} C, Pressure: {} kPa, Humidity {} %, Gas: {} ohms>".format(self.Altitude,
                                                                                                                   self.Temperature,
                                                                                                                   self.Pressure,
                                                                                                                   self.Humidity,
                                                                                                                   self.Gas)

    #Defines ACCELEROMETER object
    class ACCELEROMETER:
        
        #ACCELEROMETER constructor
        def __init__(self, X, Y, Z):
            
            self.X = X
            self.Y = Y
            self.Z = Z
        
        #Defines how ACCELEROMETER will be printed
        def __repr__(self):
            
            try:
            
                return "<X-Axis Acceleration: {:.2f} m/s^2, Y-Axis Acceleration: {:.2f} m/s^2, Z-Axis Acceleration: {:.2f} m/s^2>".format(self.X,
                                                                                                                                          self.Y,
                                                                                                                                          self.Z)

            except:

                return "<X-Axis Acceleration: {} m/s^2, Y-Axis Acceleration: {} m/s^2, Z-Axis Acceleration: {} m/s^2>".format(self.X,
                                                                                                                              self.Y,
                                                                                                                              self.Z)

    #Defines GYROSCOPE object
    class GYROSCOPE:
        
        #GYROSCOPE constructor
        def __init__(self, X, Y, Z):
            
            self.X = X
            self.Y = Y
            self.Z = Z
        
        #Defines how GYROSCOPE will be printed
        def __repr__(self):
            
            try:
                
                return "<X-Axis Rotational Velocity: {:.2f} radians/s, Y-Axis Rotational Velocity: {:.2f} radians/s, Z-Axis Rotational Velocity: {:.2f} radians/s>".format(self.X,
                                                                                                                                                                           self.Y,
                                                                                                                                                                           self.Z)
            
            except:
                
                return "<X-Axis Rotational Velocity: {} radians/s, Y-Axis Rotational Velocity: {} radians/s, Z-Axis Rotational Velocity: {} radians/s>".format(self.X,
                                                                                                                                                               self.Y,
                                                                                                                                                               self.Z)

    #Defines MAGNETOMETER object  
    class MAGNETOMETER:
        
        #MAGNETOMETER constructor
        def __init__(self, X, Y, Z):
            
            self.X = X
            self.Y = Y
            self.Z = Z
        
        #Defines how MAGNETOMETER will be printed
        def __repr__(self):
            
            try:
            
                return "<X-Axis Field Strength: {:.2f} uT, Y-Axis Field Strength: {:.2f} uT, Z-Axis Field Strength: {:.2f} uT>".format(self.X,
                                                                                                                                       self.Y,
                                                                                                                                       self.Z)
            
            except:
                
                return "<X-Axis Field Strength: {} uT, Y-Axis Field Strength: {} uT, Z-Axis Field Strength: {} uT>".format(self.X,
                                                                                                                           self.Y,
                                                                                                                           self.Z)
            
    #Defines POWER object
    class POWER:
        
        #POWER constructor
        def __init__(self, voltage, current, wattage):
            
            self.Voltage = voltage
            self.Current = current
            self.Wattage = wattage
        
        #Defines how POWER will be printed
        def __repr__(self):
            
            try:
                
                return "<Voltage: {:.3} V, Current: {:.3} A, Wattage: {:.3} W>".format(self.Voltage,
                                                                                       self.Current,
                                                                                       self.Wattage)
            
            except:
                
                return "<Voltage: {} V, Current: {} A, Wattage: {} W>".format(self.Voltage,
                                                                              self.Current,
                                                                              self.Wattage)
            
    #Defines BATTERY object
    class BATTERY:
        
        #BATTERY constructor
        def __init__(self, voltage, percentage):
            
            self.Voltage = voltage
            self.Percentage = percentage
        
        #Defines how BATTERY will be printed
        def __repr__(self):
            
            try:
                
                return "<Voltage: {:.2f} V, Percentage: {:.2f} %>".format(self.Voltage,
                                                                          self.Percentage)
            
            except:
                
                return "<Voltage: {} V, Percentage: {} %>".format(self.Voltage,
                                                                  self.Percentage)

    #Defines SOLARPANEL object
    class SOLARPANEL:
        
        #SOLARPANEL constructor
        def __init__(self, one, two, three):
            
            self.One = one
            self.Two = two
            self.Three = three
        
        #Defines how SOLARPANEL will be printed
        def __repr__(self):
            
            return "Solar Panel One: {},\nSolar Panel Two: {},\nSolar Panel Three:{}".format(self.One,
                                                                                             self.Two,
                                                                                             self.Three)

    #Defines ANALOG object
    class ANALOG:
        
        #ANALOG constructor
        def __init__(self, A0, A1, A2, A3, A4, A5, A6):
            
            self.A0 = A0
            self.A1 = A1
            self.A2 = A2
            self.A3 = A3
            self.A4 = A4
            self.A5 = A5
            self.A6 = A6
            
        #Defines how ANALOG will be printed
        def __repr__(self):
            
            try:
                
                return "<A0: {:.4f} V, A1: {:.4f} V, A2: {:.4f} V, A3: {:.4f} V, A4: {:.4f} V, A5: {:.4f} V, A6: {:.4f} V>".format(self.A0,
                                                                                                                                   self.A1,
                                                                                                                                   self.A2,
                                                                                                                                   self.A3,
                                                                                                                                   self.A4,
                                                                                                                                   self.A5,
                                                                                                                                   self.A6)

            except:
                
                return "<A0: {} V, A1: {} V, A2: {} V, A3: {} V, A4: {} V, A5: {} V, A6: {} V>".format(self.A0,
                                                                                                       self.A1,
                                                                                                       self.A2,
                                                                                                       self.A3,
                                                                                                       self.A4,
                                                                                                       self.A5,
                                                                                                       self.A6)

    #Defines AVIONICSDATA object
    class AVIONICSDATA:
        
        #AVIONICSDATA constructor
        def __init__(self, TIME, GPS, ALTIMETER, ACCELEROMETER, GYROSCOPE, MAGNETOMETER, POWERDRAW, SOLARPANEL, BATTERY, ANALOG):
            
            self.TIME = TIME
            self.GPS = GPS
            self.ALTIMETER = ALTIMETER
            self.ACCELEROMETER = ACCELEROMETER
            self.GYROSCOPE = GYROSCOPE
            self.MAGNETOMETER = MAGNETOMETER
            self.POWERDRAW = POWERDRAW
            self.SOLARPANEL = SOLARPANEL
            self.BATTERY = BATTERY
            self.ANALOG = ANALOG
        
        #Defines how AVIONICSDATA will be printed
        def __repr__(self):
            
            return "{}\nGPS: {},\nAltimeter: {},\nAccelerometer: {},\nGyroscope: {},\nMagnetometer: {},\nSystem Power Draw: {},\n{},\nBattery: {},\nAnalog: {}".format(self.TIME,
                                                                                                                                                                       self.GPS,
                                                                                                                                                                       self.ALTIMETER,
                                                                                                                                                                       self.ACCELEROMETER,
                                                                                                                                                                       self.GYROSCOPE,
                                                                                                                                                                       self.MAGNETOMETER,
                                                                                                                                                                       self.POWERDRAW,
                                                                                                                                                                       self.SOLARPANEL,
                                                                                                                                                                       self.BATTERY,
                                                                                                                                                                       self.ANALOG)
    
    #Runs setup commands for the transceiver
    def setupTransceiver():
        
        global transceiver
        
        try:
        
            transceiver = UART1
            
        except Exception as e:

            print("\n[Unable to Setup Transceiver]\n")
            errorCode(1, e)

    #Runs setup commands for the gps
    def setupGps():
        
        global gps
        
        try:
            
            gps = adafruit_gps.GPS(UART0,
                                   debug=False)
            
            gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0') #Issues parameters to GPS
            gps.send_command(b'PMTK220,1000')
            
        except Exception as e:
            
            print("\n[Unable to Setup GPS]\n")
            errorCode(3, e)

    #Runs setup commands for the altimeter
    def setupAltimeter():
        
        global bme
        
        try:
            
            bme = adafruit_bme680.Adafruit_BME680_I2C(I2C0,
                                                      address=0x77)

            bme.sea_level_pressure = 1013.25
            
            bme.pressure_oversampling = 8
            bme.temperature_oversampling = 2
            
        except Exception as e:
            
            print("\n[Unable to Setup Altimeter]\n")
            errorCode(1, e)

    #Runs setup commands for the imu
    def setupImu():
        
        global imu
        
        try:
            
            imu = LSM6DSOX(I2C0,
                           address=0x6a)
            
        except Exception as e:
            
            print("\n[Unable to Setup IMU]\n")
            errorCode(1, e)
            
    #Runs setup commands for the imu
    def setupMag():
        
        global mag
        
        try:
            
            mag = LIS3MDL(I2C0,
                          address=0x1c)
            
        except Exception as e:
            
            print("\n[Unable to Setup Magnetometer]\n")
            errorCode(1, e)

    #Runs setup commands for the current sensor that monitors power draw
    def setupPowerDraw():
        
        global powerDraw
        
        try:
            
            powerDraw = INA219(I2C0,
                               addr=0x45)

            powerDraw.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            powerDraw.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            powerDraw.bus_voltage_range = BusVoltageRange.RANGE_16V

        except Exception as e:
            
            print("\n[Unable to Connect to Power Draw Current Sensor]\n")
            errorCode(1, e)

    #Runs setup commands for the current sensors that monitor solar input
    def setupSolar():
        
        global solarOne
        global solarTwo
        global solarThree
        
        #Sets up solar set one
        try:

            solarOne = INA219(I2C0,
                              addr=0x40)

            solarOne.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            solarOne.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            solarOne.bus_voltage_range = BusVoltageRange.RANGE_16V
            
        except Exception as e:
            
            print("\n[Unable to Connect to Solar Panel One Current Sensor]\n")
            errorCode(1, e)
        
        #Sets up solar set two
        try:
            
            solarTwo = INA219(I2C0,
                              addr=0x41)

            solarTwo.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            solarTwo.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            solarTwo.bus_voltage_range = BusVoltageRange.RANGE_16V
            
        except Exception as e:
            
            print("\n[Unable to Connect to Solar Panel Two Current Sensor]\n")
            errorCode(1, e)
        
        #Sets up solar set three
        try:
            
            solarThree = INA219(I2C0,
                                addr=0x44)

            solarThree.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            solarThree.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            solarThree.bus_voltage_range = BusVoltageRange.RANGE_16V

        except Exception as e:
            
            print("\n[Unable to Connect to Solar Panel Three Current Sensor]\n")
            errorCode(1, e)

    #Runs setup commands for the battery monitor
    def setupBattery():
        
        global battery

        try:
            
            battery = adafruit_max1704x.MAX17048(I2C0,
                                                 address=0x36)
            
        except Exception as e:
            
            print("\n[Unable to Connect to Battery Sensor]\n")
            errorCode(1, e)

    #Runs setup commands for the analog to digital converter
    def setupAnalog():
        
        global adc

        try:
            
            adc = PCF.PCF8591(I2C0,
                              address=0x48)
            
        except Exception as e:
            
            print("\n[Unable to Connect to Analog to Digital Converter]\n")
            errorCode(1, e)

    #Sets up SD Card
    def setupSD():
        
        global sdcard

        try:
            
            sdcard = adafruit_sdcard.SDCard(SPI1,
                                            CS1)
            
            vfs = storage.VfsFat(sdcard)
            storage.mount(vfs, "/sd")
            
        except Exception as e:
            
            print("\n[Unable to Connect to SD Card]\n")
            errorCode(2, e)

    #getTime() handles the multiple ways the current time can be acquired
    def getTime():
        
        global gpsStatus
        
        try:
            
            gpsStatus = True
            
            #Attempts to get time from GPS
            return getGpsTime()
            
        except:
            
            try:
                
                gpsStatus = False
                
                #Attempts to get time from monotonic clock
                print("\n[Using Monotonic Time]\n")
                return round(clock.monotonic())
            
            except Exception as e:
                
                errorCode(3, e)
                
                return None

    #Acquires time from GPS
    def getGpsTime():
            
        gps.update()

        return datetime.combine(date(gps.timestamp_utc.tm_year,
                                     gps.timestamp_utc.tm_mon,
                                     gps.timestamp_utc.tm_mday),
                                time(gps.timestamp_utc.tm_hour,
                                     gps.timestamp_utc.tm_min,
                                     gps.timestamp_utc.tm_sec))
    
    #Acquires data from GPS
    def getGpsData():
        
        try:
            
            gps.update()
            
            return GPS(gps.latitude,
                       gps.longitude,
                       gps.altitude_m,
                       gps.speed_knots*(463/900),
                       gps.satellites,
                       gps.horizontal_dilution)
        
        except:
            
            return GPS(None,
                       None,
                       None,
                       None,
                       None,
                       None)

    #Acquires data from Altimeter
    def getAltimeterData():
        
        try:

            return ALTIMETER(bme.altitude,
                             bme.temperature + temperature_offset, #Applies temperature offset
                             bme.pressure,
                             bme.relative_humidity,
                             round(bme.gas))

        except:
            
            return ALTIMETER(None,
                             None,
                             None,
                             None,
                             None)

    #Acquires data from Accelerometer
    def getAccelerometerData():
        
        try:
            
            return ACCELEROMETER(imu.acceleration[0],
                                 imu.acceleration[1],
                                 imu.acceleration[2])
        
        except:
            
            return ACCELEROMETER(None,
                                 None,
                                 None)
            
    #Acquires data from Gyroscope
    def getGyroscopeData():
        
        try:
        
            return GYROSCOPE(imu.gyro[0],
                             imu.gyro[1],
                             imu.gyro[2])

        except:
            
            return GYROSCOPE(None,
                             None,
                             None)

    #Acquires data from Magnetometer
    def getMagnetometerData():
        
        try:
            
            return MAGNETOMETER(mag.magnetic[0],
                                mag.magnetic[1],
                                mag.magnetic[2])
        
        except:
            
            return MAGNETOMETER(None,
                                None,
                                None)

    #Acquires data from current sensor to determine power draw
    def getPowerDrawData():
        
        try:
            
            return POWER(powerDraw.bus_voltage,
                         (powerDraw.current / 1000),
                         abs(powerDraw.bus_voltage * (powerDraw.current / 1000)))
        
        except:
                
            return POWER(None,
                         None,
                         None)

    #Acquires data from current sensor to determine solar input
    def getSolarData():
        
        solarData = SOLARPANEL(None,
                               None,
                               None)
        
        #Pulls data for solar panel set one
        try:
            
            solarData.One = POWER(solarOne.bus_voltage,
                                  (solarOne.current / 1000),
                                  abs(solarOne.bus_voltage * (solarOne.current / 1000)))
            
        except:
            
            solarData.One =  POWER(None,
                                   None,
                                   None)
          
        #Pulls data for solar panel set Two
        try:
            
            solarData.Two = POWER(solarTwo.bus_voltage,
                                  (solarTwo.current / 1000),
                                   abs(solarTwo.bus_voltage * (solarTwo.current / 1000)))
            
        except:
            
            solarData.Two = POWER(None,
                                  None,
                                  None)
        
        #Pulls data for solar panel set Three
        try:
            
            solarData.Three = POWER(solarThree.bus_voltage,
                                    (solarThree.current / 1000),
                                     abs(solarThree.bus_voltage * (solarThree.current / 1000)))
            
        except:
            
            solarData.Three = POWER(None,
                                    None,
                                    None)
        
        return solarData

    #Acquires data from battery sensor
    def getBatteryData():
        
        try:
            
            if battery.cell_voltage >= battery_detection_threshold:
            
                return BATTERY(battery.cell_voltage,
                               battery.cell_percent)
            
            else:
                
                return BATTERY(None,
                               None)
        
        except:
            
            return BATTERY(None,
                           None)
        
    #Acquires data from Analog sensors
    def getAnalogData():
        
        try:
            
            return ANALOG(((AnalogIn(adc, PCF.A0).value / 65535) * vbus_reference_voltage),
                          ((AnalogIn(adc, PCF.A1).value / 65535) * vbus_reference_voltage),
                          ((AnalogIn(adc, PCF.A2).value / 65535) * vbus_reference_voltage),
                          ((AnalogIn(adc, PCF.A3).value / 65535) * vbus_reference_voltage),
                          ((GPIO26.value / 65536) * vbus_reference_voltage),
                          ((GPIO27.value / 65536) * vbus_reference_voltage),
                          ((GPIO28.value / 65536) * vbus_reference_voltage))
        
        except:
            
            return ANALOG(None,
                          None,
                          None,
                          None,
                          None,
                          None,
                          None)

    #Acquires data from all sensors at once
    def getAvionicsData():
        
        processLed.value = True
        
        data = AVIONICSDATA(getTime(),
                            getGpsData(),
                            getAltimeterData(),
                            getAccelerometerData(),
                            getGyroscopeData(),
                            getMagnetometerData(),
                            getPowerDrawData(),
                            getSolarData(),
                            getBatteryData(),
                            getAnalogData())
        
        processLed.value = False
        
        return data

    #Acquired and encodes data into bytes
    def getRawData():
        
        rawData = bytes()
        
        #Acquires data from all sensors
        data = getAvionicsData()
        
        print("\n" + "="*150 + "\n" + str(data) + "\n" + "="*150 + "\n")

        #Converts year, month, day, hour, minute, and second to bytes
        try:
            rawData += (data.TIME.year.to_bytes(2, 'big', signed='True') +
                        data.TIME.month.to_bytes(1, 'big', signed='True') +
                        data.TIME.day.to_bytes(1, 'big', signed='True') +
                       
                        data.TIME.hour.to_bytes(1, 'big', signed='True') +
                        data.TIME.minute.to_bytes(1, 'big', signed='True') +
                        data.TIME.second.to_bytes(1, 'big', signed='True'))
        except:
            
            try:
                rawData += (int(data.TIME)).to_bytes(7, 'big', signed='True')
            except:
                rawData += (int(clock.monotonic())).to_bytes(7, 'big', signed='True')
        
        #Converts GPS latitude, longitude, altitude, speed, satellites, and DOP to bytes
        try:
            rawData += (int(data.GPS.Latitude*10000000)).to_bytes(4, 'big', signed='True')            
        except:
            rawData += (0).to_bytes(4, 'big', signed='True')
        try:
            rawData += (int(data.GPS.Longitude*10000000)).to_bytes(4, 'big', signed='True')  
        except:
            rawData += (0).to_bytes(4, 'big', signed='True')
        try:
            rawData += (int(data.GPS.Altitude*100)).to_bytes(4, 'big', signed='True')               
        except:
            rawData += (0).to_bytes(4, 'big', signed='True')      
        try:
            rawData += (int(data.GPS.Speed*100)).to_bytes(3, 'big', signed='True')                
        except:
            rawData += (0).to_bytes(3, 'big', signed='True')
        try:
            rawData += (int(data.GPS.Satellites)).to_bytes(1, 'big', signed='True') 
        except:
            rawData += (0).to_bytes(1, 'big', signed='True')
        try:
            rawData += (int(data.GPS.DOP*100)).to_bytes(2, 'big', signed='True')   
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        
        #Converts altimeter altitude, temperature, pressure, humidty, and gas to bytes
        try:
            rawData += (int(data.ALTIMETER.Altitude*100)).to_bytes(4, 'big', signed='True')
        except:
            rawData += (0).to_bytes(4, 'big', signed='True')
        try:
            rawData += (int(data.ALTIMETER.Temperature*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')  
        try:
            rawData += (int(data.ALTIMETER.Pressure*100)).to_bytes(3, 'big', signed='True')
        except:
            rawData += (0).to_bytes(3, 'big', signed='True')
        try:
            rawData += (int(data.ALTIMETER.Humidity*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.ALTIMETER.Gas)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True') 
        
        #Convert accelerometer X, Y, and Z to bytes
        try:
            rawData += (int(data.ACCELEROMETER.X*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')   
        try:
            rawData += (int(data.ACCELEROMETER.Y*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True') 
        try:
            rawData += (int(data.ACCELEROMETER.Z*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        
        #Convert gyroscope X, Y, and Z to bytes
        try:
            rawData += (int(data.GYROSCOPE.X*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.GYROSCOPE.Y*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')  
        try:
            rawData += (int(data.GYROSCOPE.Z*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
            
        #Convert gyroscope X, Y, and Z to bytes
        try:
            rawData += (int(data.MAGNETOMETER.X*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.MAGNETOMETER.Y*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')  
        try:
            rawData += (int(data.MAGNETOMETER.Z*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
            
        #Convert power draw voltage, current, and wattage to bytes
        try:
            rawData += (int(data.POWERDRAW.Voltage*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.POWERDRAW.Current*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.POWERDRAW.Wattage*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        
        #Convert solar panel voltage, current, and wattage for panel sets one, two, and three to bytes
        try:
            rawData += (int(data.SOLARPANEL.One.Voltage*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.SOLARPANEL.One.Current*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.SOLARPANEL.One.Wattage*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.SOLARPANEL.Two.Voltage*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.SOLARPANEL.Two.Current*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.SOLARPANEL.Two.Wattage*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.SOLARPANEL.Three.Voltage*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.SOLARPANEL.Three.Current*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.SOLARPANEL.Three.Wattage*1000)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
            
        #Convert battery voltage and percentage to bytes
        try:
            rawData += (int(data.BATTERY.Voltage*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.BATTERY.Percentage*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
            
        #Convert analog voltage to bytes
        try:
            rawData += (int(data.ANALOG.A0*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.ANALOG.A1*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.ANALOG.A2*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.ANALOG.A3*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.ANALOG.A4*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.ANALOG.A5*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        try:
            rawData += (int(data.ANALOG.A6*100)).to_bytes(2, 'big', signed='True')
        except:
            rawData += (0).to_bytes(2, 'big', signed='True')
        
        return rawData

    #Grabs data from a specific recording
    def getSpecificData(specificDataNumber):
        
        try:
            
            with open("/sd/" + str(specificDataNumber), "rb") as dataFile:
                return dataFile.read()
            
        except:
            
            print("\n[Unable to Find Stored Data]\n")
            errorCode(2)
            
    #Stores data on sd card
    def storeData(data):
        
        global dataNumber
        
        try:
            
            dataNumber += 1
            
            if dataNumber >= maxDataNumber:
                
                dataNumber = 0
        
            saveState() #Updates backup information

            with open("/sd/" + str(dataNumber), "w") as dataFile:
                dataFile.write(data)
                
        except Exception as e:
            
            print("\n[Unable to Save Data to SD Card]\n")
            (2, e)

    #Handles the transmission of data through the LoRa
    def transmit(data):
        
        try:
            
            transmitLed.value = True
            
            packet = callsign.encode("ascii")
            
            if isinstance(data, bytes): #Checks if data is in byte form
                packet += data
            else:
                packet += data.encode("ascii")
            
            if len(packet) > 236: #Checks if packet is short enough for LoRa buffer
                print("[Packet is Too Long for Transmission]")
            else:
                transceiver.write(packet) #Transmits packet
                
            transmitLed.value = False
                
        except Exception as e:
            
            print("\n[Unable to Transmit Data]\n")
            errorCode(1, e)
            
            transmitLed.value = False

    #Generates the beacon signal and sends it
    def sendBeacon():
        
        global beaconSequence
        
        try:
            
            clock.sleep(0.25) #Delay to prevent altimeter oversampling
            
            if beaconSequence == 0:
                
                beaconSequence += 1
                
                dataPoint = getBatteryData().Percentage
                beaconSensor = "BAT: "
                beaconUnits = "%"
                
            elif beaconSequence == 1:
                
                beaconSequence += 1
                
                dataPoint = getGpsData().Altitude
                beaconSensor = "ALT: "
                beaconUnits = "m"
                
            elif beaconSequence == 2:
                
                beaconSequence += 1
    
                dataPoint = str(round(getGpsData().Latitude*1000)/1000) + ", " + str(round(getGpsData().Longitude*1000)/1000)
                beaconSensor = "GPS: "
                beaconUnits = ""
                
            elif beaconSequence == 3 :
                
                beaconSequence = 0
                
                dataPoint = getGpsData().Speed
                beaconSensor = "SPD: "
                beaconUnits = "m/s"
                
            else:   
                
                beaconSequence = 0
                
                dataPoint = ""
                beaconSensor = ""
                beaconUnits = ""
            
            saveState() #Updates backup information
            
            if dataPoint == None:
                dataPoint = 0
            
            try:
                dataString = str(round(dataPoint*10)/10) #Converts data to str with one decimal point
            except:
                dataString = dataPoint #Used for preformated data points
            
            transmit(" <" + beaconSensor + dataString + beaconUnits + ">")
            print("\n" + callsign + " <" + beaconSensor + dataString + beaconUnits + ">\n")
            
            clock.sleep(0.25) #Delay to prevent altimeter oversampling
            
        except Exception as e:

            print("[Beacon Data Points Could Not Be Sent]")
            errorCode(1, e)
            
            try:
                
                transmit("")
                
            except Exception as e:
                
                print("Beacon Could Not Be Sent")
                errorCode(1, e)

    #Receives and processes groundstation commands
    def commandProcessor(command):
        
        global callsign
        global beaconInterval 
        global measurementInterval
        global maxDataNumber
        global maxErrorNumber
        
        try:
            
            command = command.decode("ascii").lower().replace(" ", "") #Decodes and removes formatting from the message
            
            #Checks the message for command structure
            if command[0:7] == "avionic":
                
                command = command[7:]
                
                if command[0:4] == "sett":
                    
                    command = command[4:]
                    
                    if command[0:4] == "call":
                        
                        command = command[4:]
                        
                        callsign = command.upper()
                        
                        transmit(" {Callsign Set To: " + str(callsign) + "}")
                        print("{Callsign Set To: " + str(callsign) + "}")
                        
                        saveConfig()
                        
                    elif command[0:4] == "beac":
                        
                        command = command[4:]
                        
                        beaconInterval = int(command)
                        
                        transmit(" {Beacon Interval Set To: " + str(beaconInterval) + "}")
                        print("{Beacon Interval Set To: " + str(beaconInterval) + "}")
                        
                        saveConfig()
                        
                    elif command[0:4] == "meas":
                        
                        command = command[4:]
                        
                        measurementInterval = int(command)
                        
                        transmit(" {Measurement Interval Set To: " + str(measurementInterval) + "}")
                        print("{Measurement Interval Set To: " + str(measurementInterval) + "}")
                        
                        saveConfig()
                        
                    elif command[0:4] == "maxd":
                        
                        command = command[4:]
                        
                        maxDataNumber = int(command)
                        
                        transmit(" {Max Data Set To: " + str(maxDataNumber) + "}")
                        print("{Max Data Set To: " + str(maxDataNumber) + "}")
                        
                        saveConfig()
                        
                    elif command[0:4] == "maxe":
                        
                        command = command[4:]
                        
                        maxErrorNumber = int(command)
                        
                        transmit(" {Max Error Set To: " + str(maxErrorNumber) + "}")
                        print("{Max Error Set To: " + str(maxErrorNumber) + "}")
                        
                        saveConfig()
                        
                    elif command[0:4] == "rebt":
                        
                        transmit(" {Rebooting CubeSat}")
                        print("{Rebooting CubeSat}")
                    
                        saveState()
                        saveConfig()
                        
                        microcontroller.reset()
                        
                    elif command[0:4] == "rset":
                        
                        transmit(" {Resetting CubeSat}")
                        print("{Reseting CubeSat}")
                    
                        saveState()
                        
                        # Clears configuration
                        with open("/sd/Configuration.txt", "w") as configFile:
                            configFile.write("")
                        
                        microcontroller.reset()
                        
                    elif command[0:4] == "wipe":
                        
                        transmit(" {Wiping CubeSat}")
                        print("{Wiping CubeSat}")
                            
                        # Clears state backup
                        with open("/sd/StateBackup.txt", "w") as stateFile:
                            stateFile.write("")
                        
                        # Clears configuration
                        with open("/sd/Configuration.txt", "w") as configFile:
                            configFile.write("")
                            
                        #Clears error log
                        with open("/sd/ErrorLog.txt", "w") as errorFile:
                            errorFile.write("")
                        
                        microcontroller.reset()
                            
                    else:
                        
                        print("[Command Not Understood]")
                    
                elif command[0:4] == "tran":
                    
                    command = command[4:]
                    
                    transmit(str(' {"' + str(command) + '"}'))
                    print(str('{"' + str(command) + '"}'))
                    
                elif command[0:4] == "telm":
                    
                    command = command[4:]
                    
                    if command[0:2] == "-c":
                        
                        transmit(getRawData())
                        print("[Sending Raw Data]")
                        
                    elif command[0:2] == "-s":
                        
                        command = command[2:]
                        
                        transmit(getSpecificData(int(command)))
                        print("[Sending Specific Data]")
                        
                    else:
                        
                        print("[Command Not Understood]")
                        
                elif command[0:4] == "ping":
                    
                    command = command[4:]
                    
                    transmit(" {pong}")
                    print("{pong}")
                    
                elif command[0:4] == "pong":
                    
                    command = command[4:]
                    
                    transmit(" {ping}")
                    print("{ping}")
                    
                elif command[0:4] == "stat":
                    
                    command = command[4:]
                    
                    if command[0:2] == "-u":
                        
                        transmit(" {Uptime: " + str(clock.monotonic() - uptimeTicker) + "s}")
                        print("{Uptime: " + str(clock.monotonic() - uptimeTicker) + "s}")
                        
                    elif command[0:2] == "-m":
                        
                        transmit(" {Monotonic Time: " + str(clock.monotonic()) + "s}")
                        print("{Monotonic Time: " + str(clock.monotonic()) + "s}")
                        
                    elif command[0:2] == "-t":
                        
                        transmit(" {Time: " + str(getTime()) + "}")
                        print("{Time: " + str(getTime()) + "}")
                        
                    elif command[0:2] == "-d":
                    
                        transmit(" {Data Entries: " + str(dataNumber) + "}")
                        print("{Data Entries: " + str(dataNumber) + "}")
                        
                    elif command[0:2] == "-e":
                    
                        transmit(" {Error Entries: " + str(errorNumber) + "}")
                        print("{Error Entries: " + str(errorNumber) + "}")
                        
                    else:
                        
                        print("[Command Not Understood]")
                        
                else:
                
                    print("[Command Not Understood]")
            
            elif command[0:7] == "payload":
                
                command = command[7:]
                
                if command[0:4] == "strd":
                    transmit("StartedRecording")
                elif command[0:4] == "tpic":
                    transmit("TookPicture!")
                
                
            else:
                
                print("[Command Not Understood]")
                
        except:
          
            print("[Command Not Understood]")

    #Retrieves saved configuration from file
    def reloadConfiguration():
        
        global callsign
        global beaconInterval
        global measurementInterval
        global maxDataNumber
        global maxErrorNumber
        
        try:
                
            with open("/sd/Configuration.txt", "r") as configFile:

                configData = configFile.readlines()
                
                callsign = str(configData[0]).replace("\n", "").replace("\r", "")
                beaconInterval = int(configData[1])
                measurementInterval = int(configData[2])
                maxDataNumber = int(configData[3])
                maxErrorNumber = int(configData[4])
            
            print("[Machine Configuration Restored]")
            
        except Exception as e:
            
            print("[Unable to Reload Configuration]")
            
            saveConfig()

    #Updates configuration file
    def saveConfig():
        
        try:
        
            with open("/sd/Configuration.txt", "w") as stateFile:
                    stateFile.write(str(callsign) + "\r\n" + str(beaconInterval) + "\r\n" + str(measurementInterval) + "\r\n" + str(maxDataNumber) + "\r\n" + str(maxErrorNumber))
            
            print("[Machine Configuration Saved]")
            
        except Exception as e:
            
            print("[Unable to Save Machine Configuration]")
            errorCode(1, e)

    #Retrieves state information from file
    def resumeState():
        
        global dataNumber
        global beaconSequence
        global errorNumber
        
        try:
        
            with open("/sd/StateBackup.txt", "r") as stateFile:

                stateData = stateFile.readlines()

                dataNumber = int(stateData[0])
                beaconSequence = int(stateData[1])
                errorNumber = int(stateData[2])
            
            print("[Backup State Restored]")
            
        except Exception as e:
            
            print("[Unable to Restore Backup State]")
            
            updateDataNumber()
            saveState()
            
    def updateDataNumber():
        
        global dataNumber
        
        try:
            
            print("[Recovering Data]")
            
            fileSearch = True
            currentFile = -1
            
            while fileSearch:
                
                currentFile += 1
                
                try:
                    
                    with open("/sd/" + str(currentFile), "r") as searchFile:
                        pass
                    
                except:
                    
                    fileSearch = False
                    
            dataNumber = currentFile
            
        except:
            
            dataNumber = -1 #Line one in StateBackup.txt

    #Updates state information on file
    def saveState():
        
        try:
            
            with open("/sd/StateBackup.txt", "w") as stateFile:
                    stateFile.write(str(dataNumber) + "\r\n" + str(beaconSequence) + "\r\n" + str(errorNumber))
            
        except Exception as e:
            
            print("[Unable to Save Machine State]")
            errorCode(1, e)

    #Produces an error code on LED and saves error message in log
    def errorCode(value, message):
        
        global errorNumber
        
        print(message)
        
        try:
                
            if value >= 0:
                code = value
            else:
                code = 0
            
            errorNumber += 1
                
            if errorNumber >= maxErrorNumber:
                
                with open("/sd/ErrorLog.txt", "w") as errorFile:
                    errorFile.write("")
                
                errorNumber = 0
            
            saveState() #Updates backup information
            
            with open("/sd/ErrorLog.txt", "a") as errorFile:
                errorFile.write("[" + str(getTime()) + "] " + str(message) + "\n")
            
            for i in range(0, code):

                errorLed.value = True
                clock.sleep(0.1)
                errorLed.value = False
                clock.sleep(0.1)
                
            clock.sleep(0.25)
            
        except Exception as e:
            
            print("[Could Not Log Error]")
            print(e)
    
    def testTransmit():
        transceiver.write(("This Is A Message").encode("ascii"))

    #Acquires time from monotonic clock

    uptimeTicker = clock.monotonic()
    
    measurementTicker = clock.monotonic()
    beaconTicker = clock.monotonic()

    #Sets up SD card
    setupSD()

    #Attempts to reload configuration
    reloadConfiguration()
    resumeState()

    #Sets up GPS, locks GPS, and sets internal time
    setupGps()

    #Runs sensor setups
    setupTransceiver()
    setupAltimeter()
    setupImu()
    setupMag()
    setupPowerDraw()
    setupSolar()
    setupBattery()
    setupAnalog()

    #Main program loop
    while True:
        
        #Updates GPS
        gps.update()
        gpsLed.value = gpsStatus
        
        #Listen for transmitted commands
        command = transceiver.read(960)
        
        #Process commands if one is received
        if command != None:
            
            receiveLed.value = True
            
            commandProcessor(command)
            
            receiveLed.value = False
        
        if clock.monotonic() - measurementTicker > measurementInterval:
            
            measurementTicker = clock.monotonic()
            
            storeData(getRawData()) #Gets and stores data
            
        if clock.monotonic() - beaconTicker > beaconInterval:
            
            beaconTicker = clock.monotonic()
                
            testTransmit()

#Reboots device in the case of a critical error
except Exception as e:

    try:
        
        print("Critical Error: " + str(e))
        errorCode(1, e)
    
        microcontroller.reset()
    
    except:
    
        print("Could Not Save Error")
        microcontroller.reset()
    