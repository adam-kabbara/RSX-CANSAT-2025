import sys
from datetime import datetime, timezone
from collections import deque
import numpy as np
import random
import re
import time
import contextlib
with contextlib.redirect_stdout(None):
    from pygame import mixer
import pyqtgraph as pg
from PyQt6 import QtCore
from PyQt6.QtSerialPort import QSerialPortInfo, QSerialPort
from PyQt6.QtCore import QSize, Qt, QRect, QThread, QObject, pyqtSignal, QIODevice
from PyQt6.QtGui import QFont, QIcon, QIntValidator
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QWidget,
    QMessageBox,
    QLabel,
    QGridLayout,
    QGroupBox,
    QLineEdit,
    QVBoxLayout,
    QHBoxLayout,
    QComboBox,
)

class DynamicPlotter:

    def __init__(self, plot, title, timewindow):
        self.timewindow = timewindow
        self.databuffer = deque([0.0] * timewindow, maxlen=timewindow)
        self.x = np.linspace(-timewindow, 0, timewindow)
        self.y = np.zeros(self.databuffer.maxlen, dtype=float)

        self.plt = plot
        self.plt.setTitle(title)
        self.plt.showGrid(x=True, y=True)
        self.curve = self.plt.plot(self.x, self.y, pen=(255, 0, 0))

        self.last_time = None

    def update_plot(self, new_val):

        current_time = time.time()

        if self.last_time is None:
            time_diff = 0
        else:
            time_diff = current_time - self.last_time
            
        self.last_time = current_time

        self.databuffer.append(new_val)
        self.y[:] = self.databuffer

        self.x = np.roll(self.x, -1)
        self.x[-1] = self.x[-2] + time_diff

        self.curve.setData(self.x, self.y)
    
    def reset_plot(self):
        self.databuffer = deque([0.0] * self.timewindow, maxlen=self.timewindow)
        self.x = np.linspace(0, self.timewindow, self.timewindow)
        self.y = np.zeros(self.databuffer.maxlen, dtype=float)
        self.curve.setData(self.x, self.y)
        self.last_time = None


class GroundStationApp(QMainWindow):

    __data_received = pyqtSignal()

    def __init__(self):

        super().__init__()
        
        self.__data_received.connect(self.process_data)

        self.__CMD_GROUP_WINDOW_MAIN        = 0
        self.__CMD_GROUP_WINDOW_CHANGE_MODE = 1
        self.__CMD_GROUP_WINDOW_ADV         = 2
        self.__CMD_GROUP_WINDOW_SNC         = 3
        self.__CMD_GROUP_WINDOW_CONNECTION  = 4
        self.__recveived_data               = "NONE"
        self.__available_ports              = None
        self.__PORT_SELECTED_INFO           = None
        self.__serial                       = QSerialPort()
        self.__PORT_LABEL_OPEN              = False
        self.__TEAM_ID                      = 0
        self.__music_status                 = 0
        self.__packet_recv_count            = 0
        self.__transmission_on              = 0
        self.__serial.setBaudRate(57600)
        self.__serial.readyRead.connect(self.recv_data)

        self.setWindowTitle("CANSAT Ground Station")
        self.setWindowIcon(QIcon('media/icon.png'))

        # ------ FONTS ------ #
        button_font = QFont()
        button_font.setPointSize(14)
        button_font.setWeight(QFont.Weight.Medium)
        command_status_font = QFont()
        command_status_font.setPointSize(12)
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

        self.button_connection_group = QPushButton("CONNECTION")
        self.button_connection_group.setFont(button_font)
        self.button_connection_group.clicked.connect(lambda: self.command_group_change_buttons(self.__CMD_GROUP_WINDOW_CONNECTION))

        self.button_connect = QPushButton("OPEN/CLOSE GROUND PORT")
        self.button_connect.setFont(button_font)
        self.button_connect.clicked.connect(self.open_close_port)
        self.button_connect.hide()

        self.button_transmit = QPushButton("BEGIN/END TRANSMISSION")
        self.button_transmit.setFont(button_font)
        self.button_transmit.clicked.connect(self.toggle_transmission)

        self.button_advanced = QPushButton("ADVANCED")
        self.button_advanced.setFont(button_font)
        self.button_advanced.clicked.connect(lambda: self.command_group_change_buttons(self.__CMD_GROUP_WINDOW_ADV))

        self.button_back = QPushButton("BACK")
        self.button_back.setFont(button_font)
        self.button_back.clicked.connect(lambda: self.command_group_change_buttons(self.__CMD_GROUP_WINDOW_MAIN))
        self.button_back.hide()

        self.button_fun = QPushButton("AMBIENCE")
        self.button_fun.setFont(button_font)
        self.button_fun.clicked.connect(self.play_space_music)
        self.button_fun.hide()

        self.combo_select_port = QComboBox()
        self.combo_select_port.setPlaceholderText("SELECT PORT")
        self.combo_select_port.setFont(button_font)
        self.combo_select_port.activated.connect(self.port_selected)
        self.combo_select_port.hide()

        self.button_set_time = QPushButton("SET TIME")
        self.button_set_time.setFont(button_font)
        self.button_set_time.clicked.connect(self.send_time)
        self.button_set_time.hide()

        self.button_reset_mission = QPushButton("RESET MISSION DATA")
        self.button_reset_mission.setFont(button_font)
        self.button_reset_mission.clicked.connect(self.reset_mission)
        self.button_reset_mission.hide()

        self.button_sim_mode_enable = QPushButton("SIM MODE ENABLE")
        self.button_sim_mode_enable.setFont(button_font)
        self.button_sim_mode_enable.clicked.connect(lambda: self.change_sim_mode("ENABLE"))
        self.button_sim_mode_enable.hide()

        self.button_sim_mode_activate = QPushButton("SIM MODE ACTIVATE")
        self.button_sim_mode_activate.setFont(button_font)
        self.button_sim_mode_enable.clicked.connect(lambda: self.change_sim_mode("ACTIVATE"))
        self.button_sim_mode_activate.hide()

        self.button_sim_mode_disable = QPushButton("SIM MODE DISABLE")
        self.button_sim_mode_disable.setFont(button_font)
        self.button_sim_mode_enable.clicked.connect(lambda: self.change_sim_mode("DISABLE"))
        self.button_sim_mode_disable.hide()

        self.button_refresh_ports = QPushButton("REFRESH PORTS")
        self.button_refresh_ports.setFont(button_font)
        self.button_refresh_ports.clicked.connect(lambda: self.refresh_ports(True))
        self.button_refresh_ports.hide()

        self.button_sensor_control = QPushButton("SENSOR CONTROL")
        self.button_sensor_control.setFont(button_font)
        self.button_sensor_control.clicked.connect(lambda: self.command_group_change_buttons(self.__CMD_GROUP_WINDOW_SNC))

        self.button_altitude_cal = QPushButton("CALIBRATE ALTITUDE")
        self.button_altitude_cal.setFont(button_font)
        self.button_altitude_cal.clicked.connect(self.altitude_cal)
        self.button_altitude_cal.hide()

        self.button_activate_sensor_ex = QPushButton("ACTIVATE/DE-ACTIVATE SENSOR (NOT IMPLIMENTED)")
        self.button_activate_sensor_ex.setFont(button_font)
        self.button_activate_sensor_ex.clicked.connect(self.activate_sensor_ex)
        self.button_activate_sensor_ex.hide()

        self.button_test_connection = QPushButton("CHECK CONNECTION/GET STATUS")
        self.button_test_connection.setFont(button_font)
        self.button_test_connection.clicked.connect(self.check_remote_connection)

        self.cmd_ret_label = QLabel("GUI MSG: ")
        self.cmd_ret_label.setFont(button_font)
        self.cmd_ret_label.setStyleSheet("QLabel{color : blue; font-size: 10pt;}")
        self.cmd_ret_label.setFixedHeight(20)

        commands_layout.addWidget(self.button_connection_group)
        commands_layout.addWidget(self.combo_select_port)
        commands_layout.addWidget(self.button_connect)
        commands_layout.addWidget(self.button_refresh_ports)
        commands_layout.addWidget(self.button_test_connection)
        commands_layout.addWidget(self.button_transmit)
        commands_layout.addWidget(self.button_mode)
        commands_layout.addWidget(self.button_sensor_control)
        commands_layout.addWidget(self.button_altitude_cal)
        commands_layout.addWidget(self.button_activate_sensor_ex)
        commands_layout.addWidget(self.button_advanced)
        commands_layout.addWidget(self.button_set_time)
        commands_layout.addWidget(self.button_reset_mission)
        commands_layout.addWidget(self.button_sim_mode_enable)
        commands_layout.addWidget(self.button_sim_mode_activate)
        commands_layout.addWidget(self.button_sim_mode_disable)
        commands_layout.addWidget(self.button_fun)
        commands_layout.addWidget(self.button_back)
        commands_layout.addWidget(self.cmd_ret_label)

        commands_group_box.setFixedWidth(500)
        commands_group_box.setFixedHeight(300)

        grid_layout.addWidget(commands_group_box, 0, 0)
        grid_layout.setAlignment(commands_group_box, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        # ------ END COMMANDS GROUP ------ #

        # ------ DATA 1 GROUP ------ #
        status_group_box = QGroupBox("Status")
        status_layout = QVBoxLayout(status_group_box)

        self.label_port = QLabel()
        self.label_port.setFont(command_status_font)
        self.set_port_text_closed()

        self.label_remote_state = QLabel()
        self.label_remote_state.setFont(command_status_font)
        self.label_remote_state.setText(f'<span style="color:black;">CANSAT STATE: \
                                              </span><span style="color:RED;">UNKNOWN</span>')

        self.label_remote_mode = QLabel()
        self.label_remote_mode.setFont(command_status_font)
        self.label_remote_mode.setText(f'<span style="color:black;">CANSAT MODE: \
                                              </span><span style="color:RED;">UNKNOWN</span>')
        
        self.label_ret_msg = QLabel()
        self.label_ret_msg.setFont(command_status_font)
        self.label_ret_msg.setStyleSheet("QLabel{font-size: 10pt;}")
        self.label_ret_msg.setText(f'<span style="color:black;">RETURN MESSAGE: \
                                              </span><span style="color:RED;">NONE</span>')
        
        self.team_id_editor = QHBoxLayout()
        self.team_id_label = QLabel("TEAM ID: ")
        self.team_id_label.setFont(command_status_font)
        self.team_id_field = QLineEdit()
        self.team_id_field.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.team_id_field.setMaxLength(9)
        self.team_id_field.setText(f'{0}')
        self.team_id_field.setStyleSheet("""
            QLineEdit {
                background-color: #f0f0f0;
                border: 1px solid #cccccc;
                border-radius: 10px;
                padding: 4px;
                font-size: 14px;
            }
            
            QLineEdit:focus {
                border: 1px solid #0078d4;
                background-color: #ffffff;
            }
        """)
        int_validator = QIntValidator(self)
        self.team_id_field.setValidator(int_validator)
        self.team_id_field.editingFinished.connect(self.team_id_edited)
        self.team_id_editor.addWidget(self.team_id_label)
        self.team_id_editor.addWidget(self.team_id_field)

        status_layout.addLayout(self.team_id_editor)
        status_layout.addWidget(self.label_port)
        status_layout.addWidget(self.label_remote_mode)
        status_layout.addWidget(self.label_remote_state)
        status_layout.addWidget(self.label_ret_msg)

        status_group_box.setFixedWidth(500)
        status_group_box.setFixedHeight(300)

        grid_layout.addWidget(status_group_box, 0, 1)
        grid_layout.setAlignment(status_group_box, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        # ------ END DATA 1 GROUP ------ #

        # ------ LIVE DATA 2 GROUP ------ #
        data_group_box = QGroupBox("Live Data")
        data_layout = QVBoxLayout(data_group_box)

        self.label_mission_time = QLabel()
        self.label_mission_time.setFont(command_status_font)
        self.label_mission_time.setText(f'<span style="color:black;">MISSION TIME: \
                                              </span><span style="color:RED;">N/A</span>')
        
        self.label_gps_time = QLabel()
        self.label_gps_time.setFont(command_status_font)
        self.label_gps_time.setText(f'<span style="color:black;">GPS TIME: \
                                              </span><span style="color:RED;">N/A</span>')

        self.label_packet_count_sent = QLabel()
        self.label_packet_count_sent.setFont(command_status_font)
        self.label_packet_count_sent.setText(f'<span style="color:black;">PACKETS SENT: \
                                              </span><span style="color:RED;">N/A</span>')
        
        self.label_packet_count_recv = QLabel()
        self.label_packet_count_recv.setFont(command_status_font)
        self.label_packet_count_recv.setText(f'<span style="color:black;">PACKETS RECEIVED: \
                                              </span><span style="color:RED;">0</span>')
        
        self.label_packet_count = QLabel()
        self.label_packet_count.setFont(command_status_font)
        self.label_packet_count.setText(f'<span style="color:black;">SATELLITES: \
                                              </span><span style="color:RED;">N/A</span>')

        self.label_remote_msg = QLabel()
        self.label_remote_msg.setFont(command_status_font)
        self.label_remote_msg.setStyleSheet("QLabel{font-size: 10pt;}")
        self.label_remote_msg.setText(f'<span style="color:black;">CMD ECHO: \
                                              </span><span style="color:RED;">N/A</span>')
        
        data_layout.addWidget(self.label_mission_time)
        data_layout.addWidget(self.label_gps_time)
        data_layout.addWidget(self.label_packet_count_sent)
        data_layout.addWidget(self.label_packet_count_recv)
        data_layout.addWidget(self.label_remote_msg)

        data_group_box.setFixedWidth(500)
        data_group_box.setFixedHeight(300)

        grid_layout.addWidget(data_group_box, 0, 2)
        grid_layout.setAlignment(data_group_box, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        # ------ END DATA 2 GROUP ------ #

        # ------ GRAPH GROUP ------ #
        graphs_group_box = QGroupBox()
        graphs_layout = QGridLayout()

        self.graphs = []
        self.plotters = []

        graph_titles = [
            "Altitude + GPS Altitude [m]",
            "Temperature [°C]",
            "Pressure [kPa]",
            "Voltage [V]",
            "Gyro [deg/s]",
            "Accelerometer [deg/s^2]",
            "Magnetometer [G]",
            "Rotation [deg/s]",
            "GPS Latitude v Longitude",
        ]

        for i in range(9):
            graph = pg.PlotWidget()
            graph.setBackground('w')
            graph.setAlignment(Qt.AlignmentFlag.AlignCenter)
            graphs_layout.addWidget(graph, i // 3, i % 3)

            self.graphs.append(graph)

            # Create a DynamicPlotter for this graph
            plotter = DynamicPlotter(graph, title=graph_titles[i], timewindow=20)
            self.plotters.append(plotter)

        graphs_group_box.setLayout(graphs_layout)

        grid_layout.addWidget(graphs_group_box, 1, 0, 1, 3)
        # ------ END GRAPH GROUP ------ #
        self.showMaximized()

    def command_group_change_buttons(self, mode):
        # Hide/show buttons based on mode
        # TODO: should probably clean this
        if mode == self.__CMD_GROUP_WINDOW_ADV:
            self.button_advanced.hide()
            self.button_connection_group.hide()
            self.button_mode.hide()
            self.button_transmit.hide()
            self.button_sensor_control.hide()
            self.button_set_time.show()
            self.button_reset_mission.show()
            self.button_back.show()
            self.button_fun.show()

        elif mode == self.__CMD_GROUP_WINDOW_CHANGE_MODE:
            self.button_advanced.hide()
            self.button_connection_group.hide()
            self.button_mode.hide()
            self.button_transmit.hide()
            self.button_sensor_control.hide()
            self.button_sim_mode_enable.show()
            self.button_sim_mode_disable.show()
            self.button_sim_mode_activate.show()
            self.button_back.show()

        elif mode == self.__CMD_GROUP_WINDOW_MAIN:
            self.button_advanced.show()
            self.button_connection_group.show()
            self.button_mode.show()
            self.button_sensor_control.show()
            self.button_transmit.show()
            self.button_set_time.hide()
            self.button_reset_mission.hide()
            self.button_back.hide()
            self.button_fun.hide()
            self.button_sim_mode_activate.hide()
            self.button_sim_mode_disable.hide()
            self.button_sim_mode_enable.hide()
            self.combo_select_port.hide()
            self.button_test_connection.hide()
            self.button_connect.hide()
            self.button_refresh_ports.hide()
            self.button_altitude_cal.hide()
            self.button_activate_sensor_ex.hide()
        
        elif mode == self.__CMD_GROUP_WINDOW_SNC:
            self.button_sensor_control.hide()
            self.button_advanced.hide()
            self.button_connection_group.hide()
            self.button_mode.hide()
            self.button_transmit.hide()
            self.button_back.show()
            self.button_altitude_cal.show()
            self.button_activate_sensor_ex.show()
        
        elif mode == self.__CMD_GROUP_WINDOW_CONNECTION:
            self.button_sensor_control.hide()
            self.button_advanced.hide()
            self.button_connection_group.hide()
            self.button_mode.hide()
            self.button_transmit.hide()
            self.button_test_connection.show()
            self.button_back.show()
            self.button_connect.show()
            self.combo_select_port.clear()
            self.combo_select_port.setPlaceholderText("SELECT PORT")
            self.refresh_ports(False)
            self.combo_select_port.show()
            self.button_refresh_ports.show()
            
    def refresh_ports(self, b_print):
        self.combo_select_port.clear()
        self.combo_select_port.setPlaceholderText("SELECT PORT")
        self.__available_ports = QSerialPortInfo.availablePorts()
        for port in self.__available_ports:
            self.combo_select_port.addItem(port.portName())
        if len(self.__available_ports) == 0:
            self.combo_select_port.addItem("No available ports")
        if(b_print == True):
            self.cmd_ret_label.setText("GUI MSG: REFRESHING PORTS")

    def port_selected(self):
        if len(self.__available_ports) != 0:
            self.__PORT_SELECTED_INFO = self.__available_ports[self.combo_select_port.currentIndex()]
            self.cmd_ret_label.setText("GUI MSG: SELECTED PORT %s" % self.combo_select_port.currentText())
    
    def open_close_port(self):
        if self.__serial.isOpen() is True:
            self.__serial.close()
            if self.__serial.isOpen():
                self.cmd_ret_label.setText("GUI MSG: PORT WAS OPEN, DISCONNECTED")
                self.__PORT_LABEL_OPEN = False
                self.set_port_text_closed()
            else:
                self.cmd_ret_label.setText("GUI MSG: PORT DISCONNECT WAS UNSUCCESFUL")
        elif self.__PORT_SELECTED_INFO is not None:
            self.__serial.setPort(self.__PORT_SELECTED_INFO)
            if self.__serial.open(QIODevice.OpenModeFlag.ReadWrite):
                self.__PORT_LABEL_OPEN = True
                self.set_port_text_open()
                self.cmd_ret_label.setText("GUI MSG: OPENED PORT")
            else:
                self.cmd_ret_label.setText(f"GUI MSG: ERROR - FAILED TO OPEN PORT \
                                            {self.__PORT_SELECTED_INFO.portName()}: {self.serial.errorString()}!")
        else:
            self.cmd_ret_label.setText("GUI MSG: ERROR - SELECT PORT BEFORE ATTEMPTING TO CONNECT!")

    def check_remote_connection(self):
        self.cmd_ret_label.setText("GUI MSG: SENT TEST MESSAGE")
        self.send_data("CMD,%d,TEST,X" % self.__TEAM_ID)
    
    def send_time(self):
        utc_time = datetime.now(timezone.utc)
        time_str = utc_time.strftime("%H:%M:%S")
        self.send_data("CMD,%d,ST,%s" % (self.__TEAM_ID, time_str))
    
    def change_sim_mode(self, mode):
        self.send_data("CMD,%d,SIM,%s" % (self.__TEAM_ID, mode))
    
    def altitude_cal(self):
        self.send_data("CMD,%d,CAL,X" % self.__TEAM_ID)
    
    def activate_sensor_ex(self):
        # TODO: Need one for each device on/off
        self.send_data("CMD,%d,MEC,X:ON" % self.__TEAM_ID)
    
    def toggle_transmission(self):
        if self.__transmission_on == 0:
            self.cmd_ret_label.setText("GUI MSG: SENT TRANSMISSION ON CMD")
            self.send_data("CMD,%d,CX,ON" % self.__TEAM_ID)
            self.__transmission_on = 1
        else:
            self.cmd_ret_label.setText("GUI MSG: SENT TRANSMISSION OFF CMD")
            self.send_data("CMD,%d,CX,OFF" % self.__TEAM_ID)
            self.__transmission_on = 0

    def team_id_edited(self):
        # TODO: esc unfocuses
        self.team_id_field.clearFocus()
        self.__TEAM_ID = int(self.team_id_field.text())
        if isinstance(self.__TEAM_ID, int) and self.__TEAM_ID >= 0 and len(str(self.__TEAM_ID)) <= 10:
            self.cmd_ret_label.setText("GUI MSG: SENDING NEW TEAM ID...")
            self.send_data("CMD,%d,RESET_TEAM_ID,%d" % (self.__TEAM_ID, self.__TEAM_ID))
        else:
            self.cmd_ret_label.setText("GUI MSG: ERROR - TEAM ID MUST BE A WHOLE NUMBER UNDER 10 CHARACTERS")
            
    def send_data(self, msg):
        if self.__serial.isOpen() is True:
            self.__serial.write(msg.encode())
        elif self.__PORT_LABEL_OPEN == True:
            self.cmd_ret_label.setText("GUI MSG: CANNOT SEND DATA - PORT WAS DISCONNECTED")
            self.__PORT_LABEL_OPEN = False
            self.set_port_text_closed()
        else:
            self.cmd_ret_label.setText("GUI MSG: ERROR - OPEN PORT BEFORE SENDING DATA")

    def recv_data(self):
        while self.__serial.canReadLine():
            msg = self.__serial.readLine().data().decode().strip()
            self.__recveived_data = msg
            self.__data_received.emit()

    def process_data(self):
        # Info msg
        msg = self.__recveived_data
        if(msg.startswith('$')):
            try:
                msg_text = re.search('MSG:(.+?)', msg).group(1)
            except AttributeError:
                msg_text = "FORMAT_INCORRECT"
            try:
                mission_info = re.search('{(.+?)}', msg_text).group(1)
            except AttributeError:
                mission_info = "NONE"
            if mission_info != "NONE":
                msg_text = re.sub(r'{.+?}', '', msg_text).strip()
                new_mode, new_state, ret_team_id = mission_info.split('|')
                self.label_remote_mode.setText(f'<span style="color:black;">CANSAT MODE: \
                                              </span><span style="color:RED;">{new_mode}</span>')
                self.label_remote_state.setText(f'<span style="color:black;">CANSAT STATE: \
                                              </span><span style="color:BLUE;">{new_state}</span>')
                self.team_id_field.setText(f"{ret_team_id}")

            if msg.startswith("$IE"):
                self.label_ret_msg.setText(f'<span style="color:black;">RETURN MESSAGE: \
                                              </span><span style="color:red;">{msg_text}</span>')
            else:
                self.label_ret_msg.setText(f'<span style="color:black;">RETURN MESSAGE: \
                                              </span><span style="color:black;">{msg_text}</span>')
        else: # telemetry
            self.parse_telemetry_string(msg)
            self.label_ret_msg.setText(f'<span style="color:black;">RETURN MESSAGE: \
                                              </span><span style="color:yellow;">TELEMETRY ON, CANNOT RECEIVE MESSAGES</span>')
            self.__transmission_on = 1
    
    def reset_mission(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Icon.Warning    )
        msg_box.setWindowTitle("CONFIRM RESET")
        msg_box.setText("Are you sure you want to reset mission data?")
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)

        # Check the user's response
        response = msg_box.exec()
        if response == QMessageBox.StandardButton.Yes:
            self.__packet_recv_count = 0
            self.label_packet_count_recv.setText(f'<span style="color:black;">PACKETS RECEIVED: \
                                                </span><span style="color:RED;">{self.__packet_recv_count}</span>')
            for plotter in self.plotters:
                plotter.reset_plot()

    def set_port_text_closed(self):
         self.label_port.setText(f'<span style="color:black;">GROUND PORT: \
                                              </span><span style="color:RED;">CLOSED</span>')
        
    def set_port_text_open(self):
        open_msg = "OPEN ON: " + self.__PORT_SELECTED_INFO.portName()
        self.label_port.setText(f'<span style="color:black;">GROUND PORT: \
                                              </span><span style="color:GREEN;">{open_msg}</span>')

    def closeEvent(self, event):
        if self.__serial.isOpen() is True:
            self.__serial.close()
        
    def play_space_music(self):
        if(self.__music_status == 0):
            mixer.init()
            mixer.music.load("media/space.wav")
            mixer.music.play(loops=-1)
            self.__music_status = 1
            self.cmd_ret_label.setText("GUI MSG: AMBIENCE ON")
        elif self.__music_status == 1:
            mixer.music.pause()
            self.__music_status = 2
            self.cmd_ret_label.setText("GUI MSG: AMBIENCE OFF")
        else:
            mixer.music.unpause()
            self.__music_status = 1
            self.cmd_ret_label.setText("GUI MSG: AMBIENCE ON")
    
    def parse_telemetry_string(self, msg):
        # EXPECTED FORMAT:
        # "TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE, ALTITUDE, TEMPERATURE, PRESSURE, 
        # VOLTAGE, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y, MAG_R, MAG_P, MAG_Y, AUTO_GYRO_ROTATION_RATE, 
        # GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS, CMD_ECHO"
        self.__packet_recv_count += 1
        self.label_packet_count_recv.setText(f'<span style="color:black;">PACKETS RECEIVED: \
                                            </span><span style="color:BLUE;">{self.__packet_recv_count}</span>')

        fields = [value.strip() for value in msg.split(',')]

        if(1 <= len(fields)):
            self.label_mission_time.setText(f'<span style="color:black;">MISSION TIME: \
                                              </span><span style="color:BLUE;">{fields[1]}</span>')
        if(2 <= len(fields)):
            self.label_packet_count_sent.setText(f'<span style="color:black;">PACKETS SENT: \
                                              </span><span style="color:BLUE;">{fields[2]}</span>')
        if(3 <= len(fields)):
            self.label_remote_mode.setText(f'<span style="color:black;">CANSAT MODE: \
                                              </span><span style="color:RED;">{fields[3]}</span>')
        if(4 <= len(fields)):
            self.label_remote_state.setText(f'<span style="color:black;">CANSAT STATE: \
                                              </span><span style="color:BLUE;">{fields[4]}</span>')
        if(19 <= len(fields)):
            self.label_gps_time.setText(f'<span style="color:black;">GPS TIME: \
                                              </span><span style="color:BLUE;">{fields[19]}</span>')
        if(23 <= len(fields)):
            self.label_packet_count.setText(f'<span style="color:black;">SATELLITES: \
                                              </span><span style="color:BLUE;">{fields[23]}</span>')
        if(24 <= len(fields)):
            self.label_remote_msg.setText(f'<span style="color:black;">CMD ECHO: \
                                              </span><span style="color:BLUE;">{fields[25]}</span>')
        for i in range(5,22):
            if i == 10 or i == 11 or i == 13 or i == 14 or i == 16 or i == 17 or i == 19 or i == 20:
                continue
            if i <= 9:
                index = i - 5
            elif i == 12:
                index = 6
            elif i == 15:
                index = 7
            elif i == 18:
                index = 8
            elif i == 21:
                index = 9
            if(i <= len(fields)):
                self.plotters[index].update_plot(float(fields[i]))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = GroundStationApp()
    app.exec()