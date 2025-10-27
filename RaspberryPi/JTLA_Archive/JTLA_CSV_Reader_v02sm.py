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
        m1_target, m1_pos = [], []
        m2_target, m2_pos = [], []
        m3_target, m3_pos = [], []
        actuator = []

        with open(filename, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                timestamps.append(float(row['Time']))
                m1_target.append(float(row['M1 Target']))
                m1_pos.append(float(row['M1 Pos']))
                m2_target.append(float(row['M2 Target']))
                m2_pos.append(float(row['M2 Pos']))
                m3_target.append(float(row['M3 Target']))
                m3_pos.append(float(row['M3 Pos']))
                actuator.append(float(row['Act Length']))

        # Actual positions
        self.plot_widget.plot(timestamps, m1_pos, pen=pg.mkPen('pink', width=2), name='M1 Pos')
        self.plot_widget.plot(timestamps, m2_pos, pen=pg.mkPen('orange', width=2), name='M2 Pos')
        self.plot_widget.plot(timestamps, m3_pos, pen=pg.mkPen('green', width=2), name='M3 Pos')
        
        # Target positions (dashed lines)
        self.plot_widget.plot(timestamps, m1_target, pen=pg.mkPen('pink', width=2, style=pg.QtCore.Qt.DotLine), name='M1 Target')
        self.plot_widget.plot(timestamps, m2_target, pen=pg.mkPen('orange', width=2, style=pg.QtCore.Qt.DotLine), name='M2 Target')
        self.plot_widget.plot(timestamps, m3_target, pen=pg.mkPen('green', width=2, style=pg.QtCore.Qt.DotLine), name='M3 Target')

        # Actuator length (optional highlight)
#         self.plot_widget.plot(timestamps, actuator, pen=pg.mkPen('yellow', width=2, style=pg.QtCore.Qt.DotLine), name='Actuator Length')


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    filename = "TestData_20250609-173352.csv"  # ---UPDATE WITH CSV OF INTEREST---
    viewer = CSVPlotter(filename)
    viewer.show()
    sys.exit(app.exec_())
