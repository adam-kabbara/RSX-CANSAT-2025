from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import random
from datetime import datetime
import pytz

# FLIGHT ["TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE, ALTITUDE, TEMPERATURE, PRESSURE, VOLTAGE, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y, MAG_R, MAG_P, MAG_Y, AUTO_GYRO_ROTATION_RATE, GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS, CMD_ECHO [,,OPTIONAL_DATA]"]
packet1 =  ['001', 3600, 150, 'F', 'ASCENT', 1200.5, 22.3, 1013.25, 14.8, -0.03, 0.02, -0.01, 0.1, -0.2, 9.8, 45, -30, 90, 0.015, 120, 1198.6, 43.6426, -79.3871, 8, 'CMD']
packet2 = ['001', 3620, 151, 'F', 'ASCENT', 850.3, 24.1, 1015.6, 13.5, 0.01, -0.05, 0.00, 0.05, 0.15, 9.75, 40, -25, 85, 0.012, 140, 849.0, 43.6419, -79.3839, 7, 'CMD']
packet3 = ['001', 3640, 152, 'F', 'ASCENT', 1300.7, 23.5, 1012.0, 15.2, -0.02, 0.03, -0.01, -0.15, 0.20, 9.82, 42, -28, 88, 0.018, 160, 1302.4, 43.6431, -79.3865, 9, 'CMD']

# SIMULATION
simulation_packet = ['CMD','001', 'SIMP', 93948]

def read_cansat_file(filename, team_id):
    with open(filename, 'r') as file:
        lines = file.readlines()
    commands = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue

        command = line.replace('$', str(team_id))
        commands.append(command)
    return commands

team_id = '001' # confirmation of the format of team_id is needed
filename = 'ground_station_gui/cansat_2023_simp.txt'
simulation_packets = read_cansat_file(filename, team_id)

class Ui_GroundStation(object):
    def setupUi(self, GroundStation):
        GroundStation.setObjectName("GroundStation")
        GroundStation.resize(1144, 679)
        GroundStation.setMinimumSize(QtCore.QSize(800, 600))

        self.centralwidget = QtWidgets.QWidget(GroundStation)
        self.centralwidget.setObjectName("centralwidget")

        self.buttonSimulation = QtWidgets.QPushButton(self.centralwidget)
        self.buttonSimulation.setGeometry(QtCore.QRect(20, 20, 181, 32))
        self.buttonSimulation.setObjectName("buttonSimulation")

        self.buttonFlight = QtWidgets.QPushButton(self.centralwidget)
        self.buttonFlight.setGeometry(QtCore.QRect(200, 20, 191, 32))
        self.buttonFlight.setObjectName("buttonFlight")

        self.commandArea = QtWidgets.QTextEdit(self.centralwidget)
        self.commandArea.setGeometry(QtCore.QRect(870, 550, 261, 61))
        self.commandArea.setObjectName("commandArea")

        self.labelPackets = QtWidgets.QLabel(self.centralwidget)
        self.labelPackets.setGeometry(QtCore.QRect(420, 80, 151, 16))

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)

        self.labelPackets.setFont(font)
        self.labelPackets.setObjectName("labelPackets")

        self.labelMissionTime = QtWidgets.QLabel(self.centralwidget)
        self.labelMissionTime.setGeometry(QtCore.QRect(420, 30, 400, 30)) # Works in Windows11 system, may need adjustment in mac

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)

        self.labelMissionTime.setFont(font)
        self.labelMissionTime.setObjectName("labelMissionTime")

        self.labelGPSTime = QtWidgets.QLabel(self.centralwidget)
        self.labelGPSTime.setGeometry(QtCore.QRect(850, 30, 400, 30)) # Works in Windows11 system, may need adjustment in mac

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)

        self.labelGPSTime.setFont(font)
        self.labelGPSTime.setObjectName("labelGPSTime")

        self.labelGPSSat = QtWidgets.QLabel(self.centralwidget)
        self.labelGPSSat.setGeometry(QtCore.QRect(630, 80, 151, 16))

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)

        self.labelGPSSat.setFont(font)
        self.labelGPSSat.setObjectName("labelGPSSat")

        self.labelTeamID = QtWidgets.QLabel(self.centralwidget)
        self.labelTeamID.setGeometry(QtCore.QRect(30, 140, 151, 16))

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)

        self.labelTeamID.setFont(font)
        self.labelTeamID.setObjectName("labelTeamID")

        self.labelState = QtWidgets.QLabel(self.centralwidget)
        self.labelState.setGeometry(QtCore.QRect(30, 60, 151, 16))

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)

        self.labelState.setFont(font)
        self.labelState.setObjectName("labelState")

        self.buttonNext = QtWidgets.QPushButton(self.centralwidget)
        self.buttonNext.setGeometry(QtCore.QRect(170, 180, 151, 32))
        self.buttonNext.setObjectName("buttonNext")

        self.buttonBack = QtWidgets.QPushButton(self.centralwidget)
        self.buttonBack.setGeometry(QtCore.QRect(10, 180, 151, 32))
        self.buttonBack.setObjectName("buttonBack")

        self.graphone = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphone.setGeometry(QtCore.QRect(300, 240, 271, 191))
        self.graphone.setObjectName("Graph_One")

        self.graphtwo = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphtwo.setGeometry(QtCore.QRect(10, 480, 271, 191))
        self.graphtwo.setObjectName("Graph_Two")

        self.graphthree = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphthree.setGeometry(QtCore.QRect(10, 240, 271, 191))
        self.graphthree.setObjectName("Graph_Three")

        self.graphfour = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphfour.setGeometry(QtCore.QRect(590, 240, 271, 191))
        self.graphfour.setObjectName("Graph_Four")

        self.graphfive = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphfive.setGeometry(QtCore.QRect(300, 480, 271, 191))
        self.graphfive.setObjectName("Graph_Five")

        self.graphsix = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphsix.setGeometry(QtCore.QRect(590, 480, 271, 191))
        self.graphsix.setObjectName("Graph_Six")

        self.buttonSendCommand = QtWidgets.QPushButton(self.centralwidget)
        self.buttonSendCommand.setGeometry(QtCore.QRect(870, 630, 271, 32))
        self.buttonSendCommand.setObjectName("buttonSendCommand")

        self.labelMode = QtWidgets.QLabel(self.centralwidget)
        self.labelMode.setGeometry(QtCore.QRect(30, 100, 151, 16))

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)

        self.labelMode.setFont(font)
        self.labelMode.setObjectName("labelMode")
        GroundStation.setCentralWidget(self.centralwidget)

        self.retranslateUi(GroundStation)
        QtCore.QMetaObject.connectSlotsByName(GroundStation)

    def retranslateUi(self, GroundStation):
        _translate = QtCore.QCoreApplication.translate
        GroundStation.setWindowTitle(_translate("GroundStation", "Ground Station"))
        self.buttonSimulation.setText(_translate("GroundStation", "Simulation"))
        self.buttonFlight.setText(_translate("GroundStation", "Flight"))
        self.labelPackets.setText(_translate("GroundStation", "Number of Packets:"))
        self.labelMissionTime.setText(_translate("GroundStation", "Mission Time: "))
        self.labelGPSTime.setText(_translate("GroundStation", "GPS Time:"))
        self.labelGPSSat.setText(_translate("GroundStation", "GPS Satellites: "))
        self.labelTeamID.setText(_translate("GroundStation", "Team ID: "))
        self.labelState.setText(_translate("GroundStation", "State:"))
        self.buttonNext.setText(_translate("GroundStation", "Next"))
        self.buttonBack.setText(_translate("GroundStation", "Back"))
        self.buttonSendCommand.setText(_translate("GroundStation", "Send Command"))
        self.labelMode.setText(_translate("GroundStation", "Mode:"))

class GroundStationApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_GroundStation()
        self.ui.setupUi(self)

        # connecting buttons to methods
        self.ui.buttonSimulation.clicked.connect(self.switch_to_simulation)
        self.ui.buttonFlight.clicked.connect(self.switch_to_flight)
        self.ui.buttonSendCommand.clicked.connect(self.send_command)
        self.ui.buttonNext.clicked.connect(self.next_graph)
        self.ui.buttonBack.clicked.connect(self.previous_graph)

        self.graphs = [PlotWidget(self) for _ in range(6)]
        self.views = [self.ui.graphone, self.ui.graphtwo, self.ui.graphthree, self.ui.graphfour, self.ui.graphfive, self.ui.graphsix]
        self.xgraph_data = []
        self.ygraph_data = {
            "Altitude": [], 
            "Temperature": [], 
            "Pressure": [],
            "Voltage": [],
            "Gyro R": [],
            "Gyro P": [],
            "Gyro Y": [],
            "ACCEL_R": [],
            "ACCEL_P": [],
            "ACCEL_Y": [],
            "MAG_R" : [],
            "MAG_P" : [],
            "MAG_Y" : [],
            "AUTO_GYRO_ROTATION_RATE": [],
            "GPS_ALTITUDE" : [],
            "GPS_LATITUDE": [],
            "GPS_LONGITUDE": [] 
        }
        self.metric_names = list(self.ygraph_data.keys())
        self.place_graphs_first()
        
        self.place_graphs_first()

        # Timer for UTC and GPS time updates
        # Base time for Mission Time and GPS Time
        
        self.time_timer = QTimer()
        self.time_timer.timeout.connect(self.update_time_labels)
        self.time_timer.start(1000)
        self.time_timer.setInterval(1000)

        
        self.base_time = None  # Initially unset
        self.current_gps_time = None
        self.mission_time = None


    def place_graphs_first(self):
        global n
        n = 1
        self._place_graphs(self.metric_names[0:6], self.views)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs_first)
        self.timer.start(1000) 

    def place_graphs_second(self):
        global n
        n = 2
        self._place_graphs(self.metric_names[6:12], self.views)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs_second)
        self.timer.start(1000) 

    def place_graphs_third(self):
        global n
        n = 3
        self._place_graphs(self.metric_names[12:17], self.views[0:4])
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs_third)
        self.timer.start(1000) 

    def _place_graphs(self, metrics, views):
        for view, graph, metric in zip(views, self.graphs, metrics):
            layout = view.layout()
            if layout is None:  # Only create layout if not already assigned
                layout = QtWidgets.QVBoxLayout(view)
                view.setLayout(layout)
            layout.addWidget(graph)
            graph.setBackground('w')
            graph.addLegend()
            graph.showGrid(x=True, y=True)
            graph.setTitle(metric)

    def update_graphs_first(self):
        self.xgraph_data.append(len(self.xgraph_data) + 1)

        for metric in self.metric_names:
            self.ygraph_data[metric].append(random.randint(0, 100))

        for graph, metric in zip(self.graphs, self.metric_names[0:6]):
            graph.clear()
            graph.plot(
                self.xgraph_data,
                self.ygraph_data[metric],
                pen=pg.mkPen('r', width=4),
                name=metric
            )

    def update_graphs_second(self):
        self.xgraph_data.append(len(self.xgraph_data) + 1)
        
        for metric in self.metric_names:
            self.ygraph_data[metric].append(random.randint(0, 100))

        for graph, metric in zip(self.graphs, self.metric_names[6:12]):
            graph.clear()
            graph.plot(
                self.xgraph_data,
                self.ygraph_data[metric],
                pen=pg.mkPen('r', width=4),
                name=metric
            )

    def update_graphs_third(self):
        self.xgraph_data.append(len(self.xgraph_data) + 1)
        
        for metric in self.metric_names:
            self.ygraph_data[metric].append(random.randint(0, 100))

        for graph, metric in zip(self.graphs, self.metric_names[12:17]):
            graph.clear()
            graph.plot(
                self.xgraph_data,
                self.ygraph_data[metric],
                pen=pg.mkPen('r', width=4),
                name=metric
            )

    def switch_to_simulation(self):
        self.ui.labelMode.setText("Mode: Simulation")
        print("Switched to Simulation mode")

    def switch_to_flight(self):
        self.ui.labelMode.setText("Mode: Flight")
        print("Switched to Flight mode")

    def next_graph(self):
        if n == 1: 
            self.place_graphs_second()
        elif n == 2:
            self.place_graphs_third()
            
    def previous_graph(self):
        if n == 2: 
            self.place_graphs_first()
        elif n == 3:
            self.place_graphs_second()


    def send_command(self):
        command = self.ui.commandArea.toPlainText().strip()
        self.ui.commandArea.clear()
        
        if command.startswith("CMD") and "ST" in command:
            parts = command.split(',')
            if len(parts) == 4 and parts[2] == "ST":
                 
                time_value = parts[3]
                current_gps_time = None
                if time_value == "GPS":
                    # Set time from GPS (for simulation, use current UTC time as GPS time)
                    if self.base_time is None:
                        print ("Error: Mission Time must be set before setting GPS time.")
                        return
                    current_gps_time = datetime.now(pytz.timezone('UTC'))
                    # Calculate Mission Time converted to UTC
                    hours_offset = self.base_time.hour - current_gps_time.hour
                    current_gps_time = self.base_time.replace(hour=(self.base_time.hour + hours_offset) % 24
                        #$ minute=(self.base_time.minute) * 0,
                        #$ second=(self.base_timee.second) * 0
                    )

                    self.ui.labelGPSTime.setText(f"GPS Time: {current_gps_time.strftime('%H:%M:%S')}")

                    print(f"GPS Time set to GPS time: {current_gps_time.strftime('%H:%M:%S')}")
                    ### now = datetime.now(pytz.timezone('UTC'))
                    ### self.base_time = datetime.now(pytz.timezone('UTC'))
                    ### self.time_timer.start(1000)
                    ### print(f"Mission Time and GPS Time set to GPS: {self.base_time.strftime('%H:%M:%S')}")
                else:
                    try:
                        # Validate and set custom time
                        mission_time = datetime.strptime(time_value, '%H:%M:%S')
                        
                        self.base_time = datetime.now(pytz.timezone('UTC')).replace(
                            hour=mission_time.hour,
                            minute=mission_time.minute,
                            second=mission_time.second,
                            microsecond=0,
                            tzinfo=pytz.UTC
                        )
                        self.ui.labelMissionTime.setText(f"Mission Time: {self.base_time.strftime('%H:%M:%S')}")
                        
                        print(f"Mission Time set to: {self.base_time.strftime('%H:%M:%S')}")
                        self.time_timer.start(1000)

                    except ValueError:
                        print(f"Invalid time format: {time_value}")
            else:
                print(f"Invalid ST command format: {command}")
        else:
            print(f"Command sent: {command}")



    def update_time_labels(self):
        if self.base_time is None or self.mission_time is None:
            return
        
        now = datetime.now(pytz.timezone('UTC'))

        elapsed_mission_time = now - self.base_time
        ###### mission_time_total = self.mission_time + elapsed_mission_time
        ###### hours = mission_time_total.hour
        ###### minutes = mission_time_total.minute
        ###### seconds = mission_time_total.second
        total_seconds = int(elapsed_mission_time.total_seconds())
        # Use the total seconds from elapsed_time to calculate hours, minutes, and seconds
        hours = total_seconds // 3600
        minutes = ((total_seconds % 3600) + int(self.base_time.minute) * 60) // 60
        seconds = (total_seconds + int(self.base_time.second)) % 60

        # Format the time as HH:MM:SS
        formatted_mission_time = f"{hours:02}:{minutes:02}:{seconds:02}"

        self.ui.labelMissionTime.setText(f"Mission Time: {formatted_mission_time}")
        

        if self.current_gps_time is None:
            return
        
        elapsed_gps_time = self.mission_time.replace((self.mission_time.hour + self.hours_offset) % 24,
                    minute = self.datetime.now(pytz.timezone('UTC')).minute - self.base_time.minute,
                    second = self.datatime.now(pytz.timezone('UTC')).second - self.base_time.second
                )
                # Use the total seconds from elapsed_time to calculate hours, minutes, and seconds
        total_seconds = int(elapsed_gps_time.total_seconds())
        hours = total_seconds // 3600
        minutes = ((total_seconds % 3600) + self.base_time.minute) // 60
        seconds = (total_seconds + self.base_time.second) % 60

        # Format the time as HH:MM:SS
        formatted_gps_time = f"{hours:02}:{minutes:02}:{seconds:02}"

        self.ui.labelGPSTime.setText(f"GPS Time: {formatted_gps_time}")




if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = GroundStationApp()
    mainWindow.show()
    sys.exit(app.exec_())