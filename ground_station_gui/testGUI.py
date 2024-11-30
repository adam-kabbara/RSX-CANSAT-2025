import sys
from enum import Enum
import pyqtgraph as pg
from PyQt6 import QtSerialPort
from PyQt6.QtSerialPort import QSerialPortInfo, QSerialPort
from PyQt6.QtCore import QSize, Qt, QRect, QThread, QObject, pyqtSignal, QIODevice
from PyQt6.QtGui import QFont, QIcon
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QWidget,
    QTextEdit,
    QLabel,
    QGridLayout,
    QGroupBox,
    QLineEdit,
    QVBoxLayout,
    QComboBox,
)
"""
MAX_GRAPH_POINTS = 100

class Worker_UpdateGraphs(QObject):

    graph_update_sig = pyqtSignal(int, float, float)

    def  __init__(self, parent=None):
        super().__init__(parent=parent)
        self.graphs = []
        self.data = []
        self.max_points = MAX_GRAPH_POINTS

    def add_graph(self, graph):
        self.graphs.append(graph)
        self.data.append(([],[]))
    
    def update_graph(self, indx, x, y):
        x_data, y_data = self.data[indx]
        x_data.append(x)
        y_data.append(y)

        if len(x_data) > self.max_points:
            x_data.pop(0)
            y_data.pop(0)
        
        self.plotDataItem
"""
class GroundStationApp(QMainWindow):

    def __init__(self):

        super().__init__()

        self.__CMD_GROUP_WINDOW_MAIN        = 0
        self.__CMD_GROUP_WINDOW_CHANGE_MODE = 1
        self.__CMD_GROUP_WINDOW_ADV         = 2
        self.__available_ports              = None
        self.__PORT_SELECTED_INFO           = None
        self.__serial                       = QSerialPort()
        self.__PORT_IS_OPEN                 = False

        self.__serial.setBaudRate(57600)
        self.__serial.readyRead.connect(self.recv_data)

        self.setWindowTitle("CANSAT Ground Station")
        self.setWindowIcon(QIcon('icon.png'))

        # ------ FONTS ------ #
        button_font = QFont()
        button_font.setPointSize(14)
        button_font.setWeight(QFont.Weight.Medium)
        command_status_font = QFont()
        command_status_font.setPointSize(10)
        command_status_font.setWeight(QFont.Weight.Normal)
        # ------ FONTS ------ #

        # CENTRAL WIDGET
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        grid_layout = QGridLayout(self.central_widget)
        grid_layout.setHorizontalSpacing(10)
        grid_layout.setVerticalSpacing(30)
        grid_layout.setRowStretch(0, 1)
        grid_layout.setRowStretch(1, 3)

        # ------ COMMANDS GROUP ------ #
        commands_group_box = QGroupBox("Commands")
        commands_layout = QVBoxLayout(commands_group_box)

        self.button_mode = QPushButton("CHANGE MODE")
        self.button_mode.setFont(button_font)
        self.button_mode.clicked.connect(lambda: self.command_group_change_buttons(self.__CMD_GROUP_WINDOW_CHANGE_MODE))

        self.button_connect = QPushButton("OPEN/CLOSE GROUND CONNECTION")
        self.button_connect.setFont(button_font)
        self.button_connect.clicked.connect(self.open_close_port)

        # TODO: Lock during flight mode
        self.button_transmit = QPushButton("BEGIN/END TRANSMISSION")
        self.button_transmit.setFont(button_font)
        #button_4.clicked.connect(self.switch_to_command_4)

        self.button_advanced = QPushButton("ADVANCED")
        self.button_advanced.setFont(button_font)
        self.button_advanced.clicked.connect(lambda: self.command_group_change_buttons(self.__CMD_GROUP_WINDOW_ADV))

        self.button_back = QPushButton("BACK")
        self.button_back.setFont(button_font)
        self.button_back.clicked.connect(lambda: self.command_group_change_buttons(self.__CMD_GROUP_WINDOW_MAIN))
        self.button_back.hide()
        
        self.button_change_team_id = QPushButton("CHANGE TEAM ID")
        self.button_change_team_id.setFont(button_font)
        self.button_change_team_id.hide()

        self.combo_select_port = QComboBox()
        self.combo_select_port.setPlaceholderText("CHANGE PORT")
        self.combo_select_port.setFont(button_font)
        self.combo_select_port.activated.connect(self.port_selected)
        self.combo_select_port.hide()

        self.button_set_time = QPushButton("SET TIME")
        self.button_set_time.setFont(button_font)
        self.button_set_time.hide()

        self.button_sim_mode_enable = QPushButton("SIM MODE ENABLE")
        self.button_sim_mode_enable.setFont(button_font)
        self.button_sim_mode_enable.hide()

        self.button_refresh_ports = QPushButton("REFRESH PORTS")
        self.button_refresh_ports.setFont(button_font)
        self.button_refresh_ports.clicked.connect(self.refresh_ports)
        self.button_refresh_ports.hide()

        self.button_sim_mode_activate = QPushButton("SIM MODE ACTIVATE")
        self.button_sim_mode_activate.setFont(button_font)
        self.button_sim_mode_activate.hide()

        self.button_sim_mode_disable = QPushButton("SIM MODE DISABLE")
        self.button_sim_mode_disable.setFont(button_font)
        self.button_sim_mode_disable.hide()

        self.cmd_ret_label = QLabel("LAST CMD STATUS: ")
        self.cmd_ret_label.setFont(command_status_font)
        self.cmd_ret_label.setFixedHeight(20)

        commands_layout.addWidget(self.combo_select_port)
        commands_layout.addWidget(self.button_refresh_ports)
        commands_layout.addWidget(self.button_connect)
        commands_layout.addWidget(self.button_transmit)
        commands_layout.addWidget(self.button_mode)
        commands_layout.addWidget(self.button_advanced)
        commands_layout.addWidget(self.button_change_team_id)
        commands_layout.addWidget(self.button_set_time)
        commands_layout.addWidget(self.button_sim_mode_enable)
        commands_layout.addWidget(self.button_sim_mode_activate)
        commands_layout.addWidget(self.button_sim_mode_disable)
        commands_layout.addWidget(self.button_back)
        commands_layout.addWidget(self.cmd_ret_label)

        commands_group_box.setFixedWidth(500)
        commands_group_box.setFixedHeight(300)

        grid_layout.addWidget(commands_group_box, 0, 0)
        grid_layout.setAlignment(commands_group_box, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        # ------ END COMMANDS GROUP ------ #

        # ------ STATUS GROUP ------ #
        status_group_box = QGroupBox("Live Status")
        status_layout = QVBoxLayout(status_group_box)

        self.label_port = QLabel()
        self.label_port.setFont(button_font)
        self.label_port.setText(f'<span style="color:black;">GROUND PORT: \
                                              </span><span style="color:RED;">NOT OPEN</span>')

        self.label_connection = QLabel()
        self.label_connection.setFont(button_font)
        self.label_connection.setText(f'<span style="color:black;">CONNECTION: \
                                              </span><span style="color:RED;">NOT CONNECTED</span>')

        status_layout.addWidget(self.label_port)
        status_layout.addWidget(self.label_connection)

        status_group_box.setFixedWidth(500)
        status_group_box.setFixedHeight(300)

        grid_layout.addWidget(status_group_box, 0, 1)
        grid_layout.setAlignment(status_group_box, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        # ------ END STATUS GROUP ------ #

        # ------ LIVE DATA GROUP ------ #
        data_group_box = QGroupBox("Live Data")
        data_layout = QVBoxLayout(data_group_box)

        data_group_box.setFixedWidth(500)
        data_group_box.setFixedHeight(300)

        grid_layout.addWidget(data_group_box, 0, 2)
        grid_layout.setAlignment(data_group_box, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        # ------ END LIVE DATA GROUP ------ #

        # ------ GRAPH GROUP ------ #
        graphs_group_box = QGroupBox("Graphs")
        graphs_layout = QGridLayout()

        for i in range(9):
            graph = pg.PlotWidget()
            graph.setBackground('w')
            graph.setAlignment(Qt.AlignmentFlag.AlignCenter)
            graphs_layout.addWidget(graph, i // 3, i % 3)

        graphs_group_box.setLayout(graphs_layout)

        grid_layout.addWidget(graphs_group_box, 1, 0, 1, 3)
        # ------ END GRAPH GROUP ------ #

        self.showMaximized()

    def command_group_change_buttons(self, mode):
        # Hide/show buttons based on mode
        if mode == self.__CMD_GROUP_WINDOW_ADV:
            self.button_advanced.hide()
            self.button_connect.hide()
            self.button_mode.hide()
            self.button_transmit.hide()
            self.button_set_time.show()
            self.button_change_team_id.show()
            self.button_back.show()
            self.combo_select_port.clear()
            self.combo_select_port.setPlaceholderText("CHANGE PORT")
            self.refresh_ports()
            self.combo_select_port.show()
            self.button_refresh_ports.show()

        elif mode == self.__CMD_GROUP_WINDOW_CHANGE_MODE:
            self.button_advanced.hide()
            self.button_connect.hide()
            self.button_mode.hide()
            self.button_transmit.hide()
            self.button_sim_mode_enable.show()
            self.button_sim_mode_disable.show()
            self.button_sim_mode_activate.show()
            self.button_back.show()

        elif mode == self.__CMD_GROUP_WINDOW_MAIN:
            self.button_advanced.show()
            self.button_connect.show()
            self.button_mode.show()
            self.button_transmit.show()
            self.button_set_time.hide()
            self.button_change_team_id.hide()
            self.button_back.hide()
            self.button_sim_mode_activate.hide()
            self.button_sim_mode_disable.hide()
            self.button_sim_mode_enable.hide()
            self.combo_select_port.hide()
            self.button_refresh_ports.hide()

    def refresh_ports(self):
        self.combo_select_port.clear()
        self.combo_select_port.setPlaceholderText("CHANGE PORT")
        self.__available_ports = QSerialPortInfo.availablePorts()
        for port in self.__available_ports:
            self.combo_select_port.addItem(port.portName())
        if len(self.__available_ports) == 0:
            self.combo_select_port.addItem("No available ports")

    def port_selected(self):
        if len(self.__available_ports) != 0:
            self.__PORT_SELECTED_INFO = self.__available_ports[self.combo_select_port.currentIndex()]
            self.cmd_ret_label.setText("LAST CMD STATUS: SELECTED PORT %s" % self.combo_select_port.currentText())
    
    def open_close_port(self):
        if self.__PORT_IS_OPEN is True:
            self.cmd_ret_label.setText("LAST CMD STATUS: ATTEMPTED PORT DISCONNECT")
            if self.__serial.isOpen():
                self.__serial.close()
            self.__PORT_IS_OPEN = False
        elif self.__PORT_SELECTED_INFO is not None:
            self.__serial.setPort(self.__PORT_SELECTED_INFO)
            if self.__serial.open(QIODevice.ReadWrite):
                self.__PORT_IS_OPEN = True
                self.cmd_ret_label.setText(f"LAST CMD STATUS: SERIAL OPENED THROUGH PORT {self.__PORT_SELECTED_INFO.portName()}")
                self.label_port.setText(f'<span style="color:black;">GROUND PORT: \
                                              </span><span style="color:GREEN;">OPEN</span>')
                self.label_connection.setText(f'<span style="color:black;">CONNECTION: \
                                              </span><span style="color:YELLOW;">TESTING CONNECTION...</span>')
                self.check_remote_connection()
            else:
                self.cmd_ret_label.setText(f"LAST CMD STATUS: ERROR - FAILED TO OPEN PORT \
                                            {self.__PORT_SELECTED_INFO.portName()}: {self.serial.errorString()}!")
        else:
            self.cmd_ret_label.setText("LAST CMD STATUS: ERROR - SELECT PORT BEFORE ATTEMPTING TO CONNECT!")
    
    def check_remote_connection(self):
        print("hi2")

    def recv_data(self):
        print("hi")
    
    def switch_to_simulation(self):
        print("Switched to Simulation mode")

    def switch_to_flight(self):
        print("Switched to Flight mode")
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = GroundStationApp()
    app.exec()