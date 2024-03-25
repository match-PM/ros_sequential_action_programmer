import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import time

class LivePlotWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setupUi()

    def setupUi(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Create a PlotWidget from PyQtGraph
        self.plot_widget = pg.PlotWidget()
        layout.addWidget(self.plot_widget)

        # Initialize empty data
        self.x_data = []
        self.y_data = []

        # Start a timer to update the plot every second
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(1000)

    def update_plot(self):
        # Generate random data for demonstration
        x = time.time()  # Use time as x-axis
        y = np.random.rand()  # Random value for y-axis

        # Append data to lists
        self.x_data.append(x)
        self.y_data.append(y)

        # Plot the data
        self.plot_widget.plot(self.x_data, self.y_data, clear=True)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live Plot Example")
        self.central_widget = LivePlotWidget()
        self.setCentralWidget(self.central_widget)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
