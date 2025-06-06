import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import contextily as ctx
import geopandas as gpd
from shapely.geometry import Point

class MapWindow(QMainWindow):
    def __init__(self, lat, lon):
        super().__init__()
        self.setWindowTitle("Offline Map Viewer (contextily)")

        # Convert lat/lon to Web Mercator projection
        gdf = gpd.GeoDataFrame(
            geometry=[Point(lon, lat)],
            crs="EPSG:4326"
        ).to_crs(epsg=3857)

        # Create Matplotlib figure
        fig, ax = plt.subplots(figsize=(8, 6))
        gdf.plot(ax=ax, color="red", markersize=100)

        # Add offline map tiles (cached after first use)
        ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)

        # Center and zoom
        x, y = gdf.geometry[0].x, gdf.geometry[0].y
        ax.set_xlim(x - 500, x + 500)
        ax.set_ylim(y - 500, y + 500)
        ax.axis("off")

        # Add plot to canvas and layout
        canvas = FigureCanvas(fig)
        layout = QVBoxLayout()
        layout.addWidget(canvas)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Replace with CanSat's location
    lat = 43.6629
    lon = -79.3957

    window = MapWindow(lat, lon)
    window.resize(800, 600)
    window.show()

    sys.exit(app.exec())
