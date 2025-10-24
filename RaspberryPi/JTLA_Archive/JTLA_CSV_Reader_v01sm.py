import sys
import csv
from PyQt5 import QtWidgets
import pyqtgraph as pg

class CSVPlotter(QtWidgets.QWidget):
    def __init__(self, filename):
        super().__init__()
        self.setWindowTitle("CSV Position Data Viewer")
        self.resize(800, 600)

        # Set up plot widget
        self.plot_widget = pg.PlotWidget(title="Motor & Actuator Position Over Time")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend()

        # Layout
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

        # Load and plot data
        self.load_and_plot_csv(filename)

    def load_and_plot_csv(self, filename):
        timestamps = []
        m1, m2, m3, actuator = [], [], [], []

        with open(filename, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                timestamps.append(float(row['Time']))
                m1.append(float(row['M1 Pos']))
                m2.append(float(row['M2 Pos']))
                m3.append(float(row['M3 Pos']))
                actuator.append(float(row['Act Length']))

        self.plot_widget.plot(timestamps, m1, pen=pg.mkPen('pink', width=2), name='Motor 1')
        self.plot_widget.plot(timestamps, m2, pen=pg.mkPen('orange', width=2), name='Motor 2')
        self.plot_widget.plot(timestamps, m3, pen=pg.mkPen('g', width=2), name='Motor 3')
        self.plot_widget.plot(timestamps, actuator, pen=pg.mkPen('y', width=2, style=pg.QtCore.Qt.DashLine), name='Actuator')

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    filename = "JTLA_TestData_20250428-092016.csv"  # UPDATE THIS
    viewer = CSVPlotter(filename)
    viewer.show()
    sys.exit(app.exec_())
