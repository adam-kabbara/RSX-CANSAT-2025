from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import random

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
        self.labelMissionTime.setGeometry(QtCore.QRect(420, 30, 151, 16))

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)

        self.labelMissionTime.setFont(font)
        self.labelMissionTime.setObjectName("labelMissionTime")
        self.labelGPSTime = QtWidgets.QLabel(self.centralwidget)
        self.labelGPSTime.setGeometry(QtCore.QRect(630, 30, 151, 16))

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
        self.labelTeamID.setText(_translate("GroundStation", "Team ID: #001"))
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
        self.graph_data = {  # Store x and y for each metric
            "Altitude": [[], []], 
            "Temperature": [[], []], 
            "Pressure": [[], []],
            "Voltage": [[], []],
            "Gyro R": [[], []],
            "Gyro P": [[], []]
        }
        self.metric_names = list(self.graph_data.keys())
        self.place_graphs()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(1000)  # Update every second

    def place_graphs(self):
        views = [
            self.ui.graphone, 
            self.ui.graphtwo, 
            self.ui.graphthree,
            self.ui.graphfour, 
            self.ui.graphfive, 
            self.ui.graphsix
        ]
        for view, graph, metric in zip(views, self.graphs, self.metric_names):
            layout = QtWidgets.QVBoxLayout(view)
            layout.addWidget(graph)
            graph.setBackground('w')
            graph.addLegend()
            graph.showGrid(x=True, y=True)
            graph.setTitle(metric)  # Set title for each graph

    def update_graphs(self):
        for metric in self.metric_names:
            x = self.graph_data[metric][0]
            y = self.graph_data[metric][1]
            
            # Simulate new data (replace with actual data in real use)
            x.append(len(x) + 1)
            y.append(random.randint(0, 100))
            
            if len(x) > 100:  # Keep only the last 100 points
                x.pop(0)
                y.pop(0)

        # Update each graph
        for graph, metric in zip(self.graphs, self.metric_names):
            graph.clear()
            graph.plot(
                self.graph_data[metric][0],
                self.graph_data[metric][1],
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
        print("Next graph")

    def previous_graph(self):
        print("Previous graph")

    def send_command(self):
        command = self.ui.commandArea.toPlainText()
        print(f"Command sent: {command}")
        self.ui.commandArea.clear()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = GroundStationApp()
    mainWindow.show()
    sys.exit(app.exec_())
