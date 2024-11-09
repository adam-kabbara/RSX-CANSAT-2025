import sys
import random
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
from graph import Ui_mainbear  # Import the UI class generated from the .ui file

class GroundStationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Set up the UI from the generated Python file
        self.ui = Ui_mainbear()
        self.ui.setupUi(self)

        # Create a PyQtGraph widget and add it to a placeholder in the UI
        self.graphWidget = pg.PlotWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.graphWidget)
        self.ui.graphWidget.setLayout(layout)  # Replace `graph_placeholder` with the name in your UI

        # Set up the graph data with some initial points
        self.x_data = list(range(50))  # Initial x values (0 to 9)
        self.y_data = [2 * x + random.randint(-3, 3) for x in self.x_data]  # Initial y values with a trend and some noise

        # Set up the graph with initial data
        self.plot = self.graphWidget.plot(self.x_data, self.y_data, pen=pg.mkPen(color="white", width=3))

        # Set up a timer for updating the graph
        self.timer = QTimer()
        self.timer.setInterval(1000)  # Update interval in ms
        self.timer.timeout.connect(self.update_graph)
        self.timer.start()

    def update_graph(self):
        # Add new data for a linear graph (e.g., y = 2 * x with some noise)
        new_x = self.x_data[-1] + 1  # Next x value
        new_y = 2 * new_x + random.randint(-3, 3)  # Linear relationship with some noise

        # Append the new data
        self.x_data.append(new_x)
        self.y_data.append(new_y)
        print("x: ", self.x_data)
        print("y: ", self.y_data)

        if self.x_data[-1] > 60:
            self.x_data.pop(0)
            self.y_data.pop(0)

        # Update the graph with the new data
        self.plot.setData(self.x_data, self.y_data)

# Main function to run the application
app = QApplication(sys.argv)
window = GroundStationApp()
window.show()
sys.exit(app.exec_())