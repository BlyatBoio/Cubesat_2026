import os
import sys
import serial
import time as clock
from PySide6 import QtCore, QtWidgets
from pyqtgraph import PlotWidget
from datetime import datetime
import serial.tools.list_ports
import traceback

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
                    self.serial.flush()
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
                return self.serial.read(self.serial.in_waiting)
            except:
                error("Error reading data")

    # Welcome text in terminal     
    log("", False)   
    log("---Welcome To The Groundstation---", False)

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
        # Use a persistent byte buffer so fragmented messages are reassembled
        recv_buffer = b''
        delimiter = b'end_msg'
        while True:
            try:
                # Ensure serial object exists and is connected
                if not hasattr(radio, 'serial') or radio.serial is None or not radio.isConnected:
                    clock.sleep(0.1)
                    continue

                # Read available bytes safely
                try:
                    available = int(radio.serial.in_waiting)
                except Exception:
                    clock.sleep(0.1)
                    continue

                if available > 0:
                    raw = radio.serial.read(available)
                    if raw:
                        recv_buffer += raw

                # Prevent runaway buffer growth
                if len(recv_buffer) > 1024 * 1024:
                    error("Receive buffer overflow; trimming")
                    recv_buffer = recv_buffer[-1024 * 10:]

                # Extract complete messages terminated by delimiter
                while True:
                    idx = recv_buffer.find(delimiter)
                    if idx == -1:
                        break
                    msg_bytes = recv_buffer[:idx]
                    recv_buffer = recv_buffer[idx + len(delimiter):]

                    # Decode the message bytes safely
                    try:
                        msg = msg_bytes.decode('ascii', errors='ignore').strip()
                    except Exception:
                        try:
                            msg = msg_bytes.decode('utf-8', errors='ignore').strip()
                        except Exception:
                            msg = ''

                    if not msg:
                        continue

                    # Process the complete message
                    try:
                        p = decodeData(msg)
                        if p is None:
                            continue
                        if "ERROR" not in p:
                            log(f"{timeColor}Recieved: {inTextColor}{p}")
                            # Emit to communication stream
                            if 'gui_signals' in globals() and hasattr(gui_signals, 'communication'):
                                try:
                                    gui_signals.communication.emit(p)
                                except Exception:
                                    pass
                        else:
                            error("Recieved " + p)
                            # Also emit errors to stream
                            if 'gui_signals' in globals() and hasattr(gui_signals, 'communication'):
                                try:
                                    gui_signals.communication.emit("ERROR: " + p)
                                except Exception:
                                    pass
                    except Exception as e:
                        error(f"Error processing message: {e}")

                # small sleep to avoid busy-waiting
                clock.sleep(0.01)

            except Exception as e:
                error(f"Error in radio receiver loop: {e}")
                clock.sleep(0.5)
    
    def decodeData(packet):
        global totalGpsMeasurements, totalAltMeasurements, totalImuMeasurements, totalMagMeasurements
        try:
            newPacket = ""
            savePacketCasing = packet
            packet = packet.lower().strip()
            if not packet:
                return ""
            splitPacket = packet.split()
            pkt = splitPacket[0]
            time_value = clock.time()

            if pkt == "gps":
                if len(splitPacket) < 4:
                    error(f"GPS packet too short: {savePacketCasing}")
                    return ""
                lat = splitPacket[1]
                lon = splitPacket[2]
                alt = splitPacket[3]
                newPacket = "{}\n----------\n{}GPS Packet:\nLatitude: {} Degress\nLongitude: {} Degrees\nAltitude: {} m\n{}----------".format(
                    timeColor, inTextColor, lat, lon, alt, timeColor)
                totalGpsMeasurements += 1
                if 'gui_signals' in globals() and hasattr(gui_signals, 'gps'):
                    try:
                        gui_signals.gps.emit(lat, lon, alt, time_value)
                    except Exception:
                        pass

            elif pkt == "alt" or pkt == "lt":
                if len(splitPacket) < 6:
                    error(f"Altimeter packet too short: {savePacketCasing}")
                    return ""
                altitude = splitPacket[1]
                temperature = splitPacket[2]
                pressure = splitPacket[3]
                humidity = splitPacket[4]
                gas = splitPacket[5]
                newPacket = "{}\n----------\n{}Altimeter Packet:\nAltitude: {} m\nTemperature: {} C\nPressure: {} Kpa\nHumidity {} %\nGas: {} Ohms\n{}----------".format(
                    timeColor, inTextColor, altitude, temperature, pressure, humidity, gas, timeColor)
                totalAltMeasurements += 1
                if 'gui_signals' in globals() and hasattr(gui_signals, 'alt'):
                    try:
                        gui_signals.alt.emit(altitude, temperature, pressure, humidity, gas, time_value)
                    except Exception:
                        pass

            elif pkt == "imu":
                if len(splitPacket) < 7:
                    error(f"IMU packet too short: {savePacketCasing}")
                    return ""
                ax = splitPacket[1]
                ay = splitPacket[2]
                az = splitPacket[3]
                rx = splitPacket[4]
                ry = splitPacket[5]
                rz = splitPacket[6]
                newPacket = "{}\n----------\n{}IMU Packet:\nAcceleration X: {} m/s^2\nAcceleration Y: {} m/s^2\nAcceleration Z: {} m/s^2\nRotation X: {} Rad/s\nRotation Y: {} Rad/s\nRotation Z: {} Rad/s\n{}----------".format(
                    timeColor, inTextColor, ax, ay, az, rx, ry, rz, timeColor)
                totalImuMeasurements += 1
                if 'gui_signals' in globals() and hasattr(gui_signals, 'imu'):
                    try:
                        gui_signals.imu.emit(ax, ay, az, rx, ry, rz, time_value)
                    except Exception:
                        pass

            elif pkt == "mag":
                if len(splitPacket) < 4:
                    error(f"Magnetometer packet too short: {savePacketCasing}")
                    return ""
                mx = splitPacket[1]
                my = splitPacket[2]
                mz = splitPacket[3]
                newPacket = "{}\n----------\n{}Magnometer Packet:\nMagnetic Strength X: {} uT\nMagnetic Strength Y: {} uT\nMagnetic Strength Z: {} uT\n{}----------".format(
                    timeColor, inTextColor, mx, my, mz, timeColor)
                totalMagMeasurements += 1
                if 'gui_signals' in globals() and hasattr(gui_signals, 'mag'):
                    try:
                        gui_signals.mag.emit(mx, my, mz, time_value)
                    except Exception:
                        pass

            elif pkt == "pow":
                # Keep power packet formatting but do not update GUI here
                try:
                    newPacket = "{}\n----------\n{}Power Packet:\nPower Draw Voltage: {:.3f} V\nPower Draw Current: {:.3f} A\nPower Draw Wattage: {:.3f} W".format(
                        timeColor, inTextColor, float(splitPacket[1]), float(splitPacket[2]), float(splitPacket[3]))
                    # add solar and battery fields if present
                    if len(splitPacket) >= 7:
                        newPacket += "\nSolar 1 Voltage: {:.3f} V\nSolar 1 Current: {:.3f} A\nSolar 1 Wattage: {:.3f} W".format(float(splitPacket[4]), float(splitPacket[5]), float(splitPacket[6]))
                    # additional fields handled similarly if present
                except Exception:
                    newPacket = savePacketCasing

            elif pkt == "err":
                newPacket = "ERROR: " + savePacketCasing[4:]
            else:
                newPacket = savePacketCasing

            return newPacket
        except Exception as e:
            error(f"Error Decoding Packet:\n {e}")
            return ""

    totalGpsMeasurements = 0
    totalAltMeasurements = 0
    totalImuMeasurements = 0
    totalMagMeasurements = 0

    radioConnectThread = threading.Thread(target=connectRadio)
    commandRunnerThread = threading.Thread(target=runCommandInput)
    radioInputThread = threading.Thread(target=runRadioReciever)
    log("Initialized Threads (will start after GUI init)")
    
    class GraphWidget(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.plot_widget = PlotWidget()
            self.plot_widget.setLabel('left', 'Y axis')
            self.plot_widget.setLabel('bottom', 'X axis')
            
            layout = QtWidgets.QVBoxLayout(self)
            layout.addWidget(self.plot_widget)
        
        def plot_data(self, x_data, y_data):
            self.plot_widget.plot(x_data, y_data, pen='b')
            
    class gpsWidget(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            
            # Data lists for storing history
            self.latitude_data = []
            self.longitude_data = []
            self.altitude_data = []
            self.time_data = []
            
            # Create graphs
            self.latitude_plot = PlotWidget()
            self.latitude_plot.setLabel('left', 'Latitude (°)')
            self.latitude_plot.setLabel('bottom', 'Time (s)')
            self.latitude_plot.setTitle('Latitude')
            
            self.longitude_plot = PlotWidget()
            self.longitude_plot.setLabel('left', 'Longitude (°)')
            self.longitude_plot.setLabel('bottom', 'Time (s)')
            self.longitude_plot.setTitle('Longitude')
            
            self.altitude_plot = PlotWidget()
            self.altitude_plot.setLabel('left', 'Altitude (m)')
            self.altitude_plot.setLabel('bottom', 'Time (s)')
            self.altitude_plot.setTitle('Altitude')
            
            # Create text displays
            self.latitude_text = QtWidgets.QLineEdit()
            self.latitude_text.setReadOnly(True)
            self.latitude_text.setText("Latitude: --")
            self.latitude_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.longitude_text = QtWidgets.QLineEdit()
            self.longitude_text.setReadOnly(True)
            self.longitude_text.setText("Longitude: --")
            self.longitude_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.altitude_text = QtWidgets.QLineEdit()
            self.altitude_text.setReadOnly(True)
            self.altitude_text.setText("Altitude: --")
            self.altitude_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            # Create layout
            main_layout = QtWidgets.QGridLayout(self)
            
            # Top row: graphs
            main_layout.addWidget(self.latitude_plot, 0, 0)
            main_layout.addWidget(self.longitude_plot, 0, 1)
            main_layout.addWidget(self.altitude_plot, 0, 2)
            
            # Bottom row: text displays
            main_layout.addWidget(self.latitude_text, 1, 0)
            main_layout.addWidget(self.longitude_text, 1, 1)
            main_layout.addWidget(self.altitude_text, 1, 2)
            
            self.setLayout(main_layout)
        
        def update_gps_data(self, latitude, longitude, altitude, time_value):
            """Update GPS data and refresh graphs"""
            try:
                # Convert to float
                lat = float(latitude)
                lon = float(longitude)
                alt = float(altitude)
                
                # Add to data lists
                self.latitude_data.append(lat)
                self.longitude_data.append(lon)
                self.altitude_data.append(alt)
                self.time_data.append(time_value)
                
                # Update text displays
                self.latitude_text.setText(f"Latitude: {lat:.6f}°")
                self.longitude_text.setText(f"Longitude: {lon:.6f}°")
                self.altitude_text.setText(f"Altitude: {alt:.2f} m")
                
                # Update graphs
                self.latitude_plot.clear()
                self.latitude_plot.plot(self.time_data, self.latitude_data, pen='cyan')
                
                self.longitude_plot.clear()
                self.longitude_plot.plot(self.time_data, self.longitude_data, pen='yellow')
                
                self.altitude_plot.clear()
                self.altitude_plot.plot(self.time_data, self.altitude_data, pen='magenta')
                
            except Exception as e:
                print(f"Error updating GPS data: {e}")

    class altimeterWidget(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            
            # Data lists for storing history
            self.altitude_data = []
            self.temperature_data = []
            self.pressure_data = []
            self.humidity_data = []
            self.gas_data = []
            self.time_data = []
            
            # Create graphs (5 separate graphs)
            self.altitude_plot = PlotWidget()
            self.altitude_plot.setLabel('left', 'Altitude (m)')
            self.altitude_plot.setLabel('bottom', 'Time (s)')
            self.altitude_plot.setTitle('Altitude')
            
            self.temperature_plot = PlotWidget()
            self.temperature_plot.setLabel('left', 'Temperature (°C)')
            self.temperature_plot.setLabel('bottom', 'Time (s)')
            self.temperature_plot.setTitle('Temperature')
            
            self.pressure_plot = PlotWidget()
            self.pressure_plot.setLabel('left', 'Pressure (kPa)')
            self.pressure_plot.setLabel('bottom', 'Time (s)')
            self.pressure_plot.setTitle('Pressure')
            
            self.humidity_plot = PlotWidget()
            self.humidity_plot.setLabel('left', 'Humidity (%)')
            self.humidity_plot.setLabel('bottom', 'Time (s)')
            self.humidity_plot.setTitle('Humidity')
            
            self.gas_plot = PlotWidget()
            self.gas_plot.setLabel('left', 'Gas (Ω)')
            self.gas_plot.setLabel('bottom', 'Time (s)')
            self.gas_plot.setTitle('Gas')
            
            # Create text displays
            self.altitude_text = QtWidgets.QLineEdit()
            self.altitude_text.setReadOnly(True)
            self.altitude_text.setText("Altitude: -- m")
            self.altitude_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.temperature_text = QtWidgets.QLineEdit()
            self.temperature_text.setReadOnly(True)
            self.temperature_text.setText("Temperature: -- °C")
            self.temperature_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.pressure_text = QtWidgets.QLineEdit()
            self.pressure_text.setReadOnly(True)
            self.pressure_text.setText("Pressure: -- kPa")
            self.pressure_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.humidity_text = QtWidgets.QLineEdit()
            self.humidity_text.setReadOnly(True)
            self.humidity_text.setText("Humidity: -- %")
            self.humidity_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.gas_text = QtWidgets.QLineEdit()
            self.gas_text.setReadOnly(True)
            self.gas_text.setText("Gas: -- Ω")
            self.gas_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            # Create layout
            main_layout = QtWidgets.QGridLayout(self)
            
            # Top row: graphs
            main_layout.addWidget(self.altitude_plot, 0, 0)
            main_layout.addWidget(self.temperature_plot, 0, 1)
            main_layout.addWidget(self.pressure_plot, 0, 2)
            main_layout.addWidget(self.humidity_plot, 0, 3)
            main_layout.addWidget(self.gas_plot, 0, 4)
            
            # Bottom row: text displays
            main_layout.addWidget(self.altitude_text, 1, 0)
            main_layout.addWidget(self.temperature_text, 1, 1)
            main_layout.addWidget(self.pressure_text, 1, 2)
            main_layout.addWidget(self.humidity_text, 1, 3)
            main_layout.addWidget(self.gas_text, 1, 4)
            
            self.setLayout(main_layout)
        
        def update_altimeter_data(self, altitude, temperature, pressure, humidity, gas, time_value):
            """Update altimeter data and refresh graphs"""
            try:
                # Convert to float
                alt = float(altitude)
                temp = float(temperature)
                pres = float(pressure)
                hum = float(humidity)
                gas_val = float(gas)
                
                # Add to data lists
                self.altitude_data.append(alt)
                self.temperature_data.append(temp)
                self.pressure_data.append(pres)
                self.humidity_data.append(hum)
                self.gas_data.append(gas_val)
                self.time_data.append(time_value)
                
                # Update text displays
                self.altitude_text.setText(f"Altitude: {alt:.2f} m")
                self.temperature_text.setText(f"Temperature: {temp:.2f} °C")
                self.pressure_text.setText(f"Pressure: {pres:.2f} kPa")
                self.humidity_text.setText(f"Humidity: {hum:.2f} %")
                self.gas_text.setText(f"Gas: {gas_val:.2f} Ω")
                
                # Update graphs
                self.altitude_plot.clear()
                self.altitude_plot.plot(self.time_data, self.altitude_data, pen='cyan')
                
                self.temperature_plot.clear()
                self.temperature_plot.plot(self.time_data, self.temperature_data, pen='red')
                
                self.pressure_plot.clear()
                self.pressure_plot.plot(self.time_data, self.pressure_data, pen='yellow')
                
                self.humidity_plot.clear()
                self.humidity_plot.plot(self.time_data, self.humidity_data, pen='green')
                
                self.gas_plot.clear()
                self.gas_plot.plot(self.time_data, self.gas_data, pen='magenta')
                
            except Exception as e:
                print(f"Error updating altimeter data: {e}")

    class imuWidget(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            
            # Data lists for storing history
            self.accel_x_data = []
            self.accel_y_data = []
            self.accel_z_data = []
            self.rot_x_data = []
            self.rot_y_data = []
            self.rot_z_data = []
            self.time_data = []
            
            # Create acceleration graph
            self.accel_plot = PlotWidget()
            self.accel_plot.setLabel('left', 'Acceleration (m/s²)')
            self.accel_plot.setLabel('bottom', 'Time (s)')
            self.accel_plot.setTitle('Acceleration')
            
            # Create rotation graph
            self.rotation_plot = PlotWidget()
            self.rotation_plot.setLabel('left', 'Rotation (Rad/s)')
            self.rotation_plot.setLabel('bottom', 'Time (s)')
            self.rotation_plot.setTitle('Rotation')
            
            # Create text displays
            self.accel_x_text = QtWidgets.QLineEdit()
            self.accel_x_text.setReadOnly(True)
            self.accel_x_text.setText("Accel X: -- m/s²")
            self.accel_x_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.accel_y_text = QtWidgets.QLineEdit()
            self.accel_y_text.setReadOnly(True)
            self.accel_y_text.setText("Accel Y: -- m/s²")
            self.accel_y_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.accel_z_text = QtWidgets.QLineEdit()
            self.accel_z_text.setReadOnly(True)
            self.accel_z_text.setText("Accel Z: -- m/s²")
            self.accel_z_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.rot_x_text = QtWidgets.QLineEdit()
            self.rot_x_text.setReadOnly(True)
            self.rot_x_text.setText("Rot X: -- Rad/s")
            self.rot_x_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.rot_y_text = QtWidgets.QLineEdit()
            self.rot_y_text.setReadOnly(True)
            self.rot_y_text.setText("Rot Y: -- Rad/s")
            self.rot_y_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.rot_z_text = QtWidgets.QLineEdit()
            self.rot_z_text.setReadOnly(True)
            self.rot_z_text.setText("Rot Z: -- Rad/s")
            self.rot_z_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            # Create layout
            main_layout = QtWidgets.QGridLayout(self)
            
            # Top row: graphs
            main_layout.addWidget(self.accel_plot, 0, 0, 1, 3)
            main_layout.addWidget(self.rotation_plot, 0, 3, 1, 3)
            
            # Bottom row: text displays in vertical lists
            accel_layout = QtWidgets.QVBoxLayout()
            accel_layout.addWidget(self.accel_x_text)
            accel_layout.addWidget(self.accel_y_text)
            accel_layout.addWidget(self.accel_z_text)
            
            rot_layout = QtWidgets.QVBoxLayout()
            rot_layout.addWidget(self.rot_x_text)
            rot_layout.addWidget(self.rot_y_text)
            rot_layout.addWidget(self.rot_z_text)
            
            main_layout.addLayout(accel_layout, 1, 0, 1, 3)
            main_layout.addLayout(rot_layout, 1, 3, 1, 3)
            
            self.setLayout(main_layout)
        
        def update_imu_data(self, accel_x, accel_y, accel_z, rot_x, rot_y, rot_z, time_value):
            """Update IMU data and refresh graphs"""
            try:
                # Convert to float
                ax = float(accel_x)
                ay = float(accel_y)
                az = float(accel_z)
                rx = float(rot_x)
                ry = float(rot_y)
                rz = float(rot_z)
                
                # Add to data lists
                self.accel_x_data.append(ax)
                self.accel_y_data.append(ay)
                self.accel_z_data.append(az)
                self.rot_x_data.append(rx)
                self.rot_y_data.append(ry)
                self.rot_z_data.append(rz)
                self.time_data.append(time_value)
                
                # Update text displays
                self.accel_x_text.setText(f"Accel X: {ax:.3f} m/s²")
                self.accel_y_text.setText(f"Accel Y: {ay:.3f} m/s²")
                self.accel_z_text.setText(f"Accel Z: {az:.3f} m/s²")
                self.rot_x_text.setText(f"Rot X: {rx:.3f} Rad/s")
                self.rot_y_text.setText(f"Rot Y: {ry:.3f} Rad/s")
                self.rot_z_text.setText(f"Rot Z: {rz:.3f} Rad/s")
                
                # Update graphs with all three axes
                self.accel_plot.clear()
                self.accel_plot.plot(self.time_data, self.accel_x_data, pen='red', name='X')
                self.accel_plot.plot(self.time_data, self.accel_y_data, pen='green', name='Y')
                self.accel_plot.plot(self.time_data, self.accel_z_data, pen='blue', name='Z')
                self.accel_plot.addLegend()
                
                self.rotation_plot.clear()
                self.rotation_plot.plot(self.time_data, self.rot_x_data, pen='red', name='X')
                self.rotation_plot.plot(self.time_data, self.rot_y_data, pen='green', name='Y')
                self.rotation_plot.plot(self.time_data, self.rot_z_data, pen='blue', name='Z')
                self.rotation_plot.addLegend()
                
            except Exception as e:
                print(f"Error updating IMU data: {e}")

    class magnetometerWidget(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            
            # Data lists for storing history
            self.mag_x_data = []
            self.mag_y_data = []
            self.mag_z_data = []
            self.time_data = []
            
            # Create graph
            self.mag_plot = PlotWidget()
            self.mag_plot.setLabel('left', 'Magnetic Strength (μT)')
            self.mag_plot.setLabel('bottom', 'Time (s)')
            self.mag_plot.setTitle('Magnetometer')
            
            # Create text displays
            self.mag_x_text = QtWidgets.QLineEdit()
            self.mag_x_text.setReadOnly(True)
            self.mag_x_text.setText("Mag X: -- μT")
            self.mag_x_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.mag_y_text = QtWidgets.QLineEdit()
            self.mag_y_text.setReadOnly(True)
            self.mag_y_text.setText("Mag Y: -- μT")
            self.mag_y_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            self.mag_z_text = QtWidgets.QLineEdit()
            self.mag_z_text.setReadOnly(True)
            self.mag_z_text.setText("Mag Z: -- μT")
            self.mag_z_text.setStyleSheet("background-color: #2b2b2b; color: #00ff00;")
            
            # Create layout
            main_layout = QtWidgets.QGridLayout(self)
            
            # Top: graph spanning all columns
            main_layout.addWidget(self.mag_plot, 0, 0, 1, 3)
            
            # Bottom: text displays in vertical list
            text_layout = QtWidgets.QVBoxLayout()
            text_layout.addWidget(self.mag_x_text)
            text_layout.addWidget(self.mag_y_text)
            text_layout.addWidget(self.mag_z_text)
            
            main_layout.addLayout(text_layout, 1, 0, 1, 3)
            
            self.setLayout(main_layout)
        
        def update_magnetometer_data(self, mag_x, mag_y, mag_z, time_value):
            """Update magnetometer data and refresh graph"""
            try:
                # Convert to float
                mx = float(mag_x)
                my = float(mag_y)
                mz = float(mag_z)
                
                # Add to data lists
                self.mag_x_data.append(mx)
                self.mag_y_data.append(my)
                self.mag_z_data.append(mz)
                self.time_data.append(time_value)
                
                # Update text displays
                self.mag_x_text.setText(f"Mag X: {mx:.2f} μT")
                self.mag_y_text.setText(f"Mag Y: {my:.2f} μT")
                self.mag_z_text.setText(f"Mag Z: {mz:.2f} μT")
                
                # Update graph with all three axes
                self.mag_plot.clear()
                self.mag_plot.plot(self.time_data, self.mag_x_data, pen='red', name='X')
                self.mag_plot.plot(self.time_data, self.mag_y_data, pen='green', name='Y')
                self.mag_plot.plot(self.time_data, self.mag_z_data, pen='blue', name='Z')
                self.mag_plot.addLegend()
                
            except Exception as e:
                print(f"Error updating magnetometer data: {e}")
            
    class commandInputWidget(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            
            # Use QTextEdit instead of QLabel for scrolling capability
            self.comunicationStream = QtWidgets.QTextEdit()
            self.comunicationStream.setReadOnly(True)  # Prevent user editing
            self.comunicationStream.setPlainText("")
            
            self.commandInput = QtWidgets.QTextEdit()
            self.sendCommandButton = QtWidgets.QPushButton("Send Command")
            
            self.sendCommandButton.setMaximumSize(800, 100)
            self.commandInput.setMaximumSize(800, 100)
            self.comunicationStream.setMinimumSize(200, 600)
            
            self.comunicationStream.setStyleSheet("color: white; background-color: #1e1e1e;")
            self.commandInput.setStyleSheet("background-color: #2b2b2b; color: #00ff00; border: 1px solid #555;")
            self.sendCommandButton.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    padding: 5px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
                QPushButton:pressed {
                    background-color: #3d8b40;
                }
            """)
                        
            self.mainLayout = QtWidgets.QGridLayout(self)
            self.mainLayout.addWidget(self.comunicationStream, 0, 0)
            self.mainLayout.addWidget(self.commandInput, 1, 0)
            self.mainLayout.addWidget(self.sendCommandButton, 2, 0)
            
            self.setLayout(self.mainLayout)

            self.sendCommandButton.clicked.connect(self.sendCommand)
        
        def keyPressEvent(self, event):
            if event.key() == QtCore.Qt.Key_Return:
                self.sendCommand()
            else:
                super().keyPressEvent(event)
        
        def sendCommand(self):
            command_text = self.commandInput.toPlainText()
            runCommand(command_text)
            # Append to communication stream and scroll to bottom
            self.comunicationStream.append(">>> " + command_text)
            self.commandInput.clear()
            # Scroll to bottom
            scrollbar = self.comunicationStream.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
        
        def appendRecievedCommand(self, recievedString):
            """Append received message and auto-scroll to bottom"""
            self.comunicationStream.append(recievedString)
            # Scroll to bottom
            scrollbar = self.comunicationStream.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
    
    app = QtWidgets.QApplication([])
    
    commandWidget = commandInputWidget()

    # Create the display widgets (do not show individually)
    gpsDisplay = gpsWidget()
    altDisplay = altimeterWidget()
    imuDisplay = imuWidget()
    magDisplay = magnetometerWidget()

    # Build the main window: left command terminal (~1/5), right vertical stack (~4/5)
    main_window = QtWidgets.QWidget()
    main_layout = QtWidgets.QHBoxLayout(main_window)

    # Left: command widget (fixed minimum width)
    commandWidget.setMinimumWidth(250)
    main_layout.addWidget(commandWidget)

    # Right: vertical stack of graph sections
    right_container = QtWidgets.QWidget()
    right_layout = QtWidgets.QVBoxLayout(right_container)
    right_layout.setSpacing(8)
    right_layout.setContentsMargins(0,0,0,0)

    # Add graph displays stacked vertically; allow them to expand equally
    right_layout.addWidget(gpsDisplay, 1)
    right_layout.addWidget(altDisplay, 1)
    right_layout.addWidget(imuDisplay, 1)
    right_layout.addWidget(magDisplay, 1)

    main_layout.addWidget(right_container)
    # Give left: right a 1:4 stretch ratio
    main_layout.setStretch(0, 1)
    main_layout.setStretch(1, 4)

    main_window.setLayout(main_layout)
    main_window.resize(1200, 800)
    main_window.show()
    
    # Create GUI signal dispatcher and connect worker threads to GUI update methods
    class GuiSignals(QtCore.QObject):
        gps = QtCore.Signal(object, object, object, object)
        alt = QtCore.Signal(object, object, object, object, object, object)
        imu = QtCore.Signal(object, object, object, object, object, object, object)
        mag = QtCore.Signal(object, object, object, object)
        communication = QtCore.Signal(str)  # Signal to append to communication stream

    gui_signals = GuiSignals()

    gui_signals.gps.connect(gpsDisplay.update_gps_data)
    gui_signals.alt.connect(altDisplay.update_altimeter_data)
    gui_signals.imu.connect(imuDisplay.update_imu_data)
    gui_signals.mag.connect(magDisplay.update_magnetometer_data)
    gui_signals.communication.connect(commandWidget.appendRecievedCommand)

    # Now that GUI objects exist on the main thread, start background threads
    radioConnectThread.start()
    # do not join here to keep UI responsive
    commandRunnerThread.start()
    radioInputThread.start()
    log("Started Threads")
    
    sys.exit(app.exec())

except Exception as e:
    error("Critical Error:")
    traceback.print_exception(type(e), e, e.__traceback__)