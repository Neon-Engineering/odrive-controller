import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QGraphicsRectItem
from PyQt5.QtGui import QImageReader, QImage
import pyqtgraph as pg

class ActuatorVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Actuator Visualizer with Background")

        # Layout setup
        main_layout = QHBoxLayout(self)
        plot_layout = QVBoxLayout()
        gui_layout = QVBoxLayout()

        # Create PyQtGraph plot view
        self.view = pg.GraphicsLayoutWidget()
        self.plot = self.view.addViewBox()
        self.plot.setAspectLocked(True)
        self.plot.setRange(xRange=(0, 100), yRange=(0, 20))  # Tune to match image proportions

        # Load and rotate background image
        reader = QImageReader("JTLA_Dark_Large.jpg")  # Replace with actual image name
        qimage = reader.read()
        if qimage.isNull():
            print("Failed to load image.")
            sys.exit(1)

        # Ensure consistent format (RGBA8888)
        qimage = qimage.convertToFormat(QImage.Format_RGBA8888)

        ptr = qimage.bits()
        ptr.setsize(qimage.byteCount())
        image_array = np.array(ptr).reshape(qimage.height(), qimage.width(), 4)
        
        self.bg_image = pg.ImageItem(image_array)
        self.bg_image.setZValue(-1)

        # Scale to fit view box range (e.g. 0–100 x, 0–20 y)
        img_width = image_array.shape[1]
        img_height = image_array.shape[0]
        scale_x = 10 / img_width
        scale_y = 10 / img_height

        from PyQt5.QtGui import QTransform
        transform = QTransform().scale(scale_x, scale_y)
        self.bg_image.setTransform(transform)

        self.plot.addItem(self.bg_image)


        # Rotate image 90° clockwise
        image_array = np.rot90(image_array, k=3)

        # Add image to plot
        self.bg_image = pg.ImageItem(image_array)
        self.bg_image.setZValue(-1)  # Ensure it's in the background
        self.plot.addItem(self.bg_image)

        # --- Static stacked bar (adjust as needed) ---
        self.bars = []
        section_lengths = [20, 30, 10]  # Widths of segments
        section_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

        start_x = 40   # X position where the stacked bar begins
        bar_y = 5      # Vertical center of the bar
        bar_height = 6  # Thickness of each segment

        for width, color in zip(section_lengths, section_colors):
            bar = QGraphicsRectItem(start_x, bar_y, width, bar_height)
            bar.setBrush(pg.mkBrush(color))
            bar.setPen(pg.mkPen('black'))
            self.plot.addItem(bar)
            self.bars.append(bar)
            start_x += width

        # Add the view to the layout
        plot_layout.addWidget(self.view)
        main_layout.addLayout(plot_layout)

        # GUI controls
        for label in ["Motor 1", "Motor 2", "Motor 3", "Send", "Home"]:
            button = QPushButton(label)
            button.setMinimumHeight(50)
            gui_layout.addWidget(button)

        gui_layout.addStretch()
        main_layout.addLayout(gui_layout)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ActuatorVisualizer()
    window.resize(1000, 600)
    window.show()
    sys.exit(app.exec_())
