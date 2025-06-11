import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Add this import
import sys
import re


def generate_plot(excel_path, y_col, x_col, z_col, x_label, y_label, z_label, title, output_image, section_col=None):
    # Read the Excel file
    df = pd.read_excel(excel_path)
    # Get column names by index
    x_col_name = df.columns[x_col]  # GPS Latitude
    y_col_name = df.columns[y_col]  # GPS Longitude
    z_col_name = df.columns[z_col]  # GPS Altitude
    section_col_name = df.columns[section_col] if section_col is not None else None
    # Remove rows with missing or invalid x/y/z values
    cols_to_keep = [x_col_name, y_col_name, z_col_name] + ([section_col_name] if section_col_name else [])
    df = df[cols_to_keep].dropna()
    x = df[x_col_name]
    y = df[y_col_name]
    z = df[z_col_name]
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, marker='o', label=f'{z_label} vs {x_label} vs {y_label}')
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_zlabel(z_label)
    ax.set_title(title)
    ax.legend()
    plt.tight_layout()
    plt.savefig(output_image)
    print(f"Plot saved as {output_image}")
    plt.show()


def main():
    if len(sys.argv) not in (10, 11):
        print("Usage: python generate_sensor_plot.py <excel_path> <y_col> <x_col> <z_col> <x_label> <y_label> <z_label> <title> <output_image> [section_col]")
        print("Example: python generate_sensor_plot.py data.xlsx 0 1 2 'Latitude' 'Longitude' 'Altitude' '3D GPS Plot' output.png [section_col]")
        sys.exit(1)
    excel_path = sys.argv[1]
    y_col = int(sys.argv[2])
    x_col = int(sys.argv[3])
    z_col = int(sys.argv[4])
    x_label = sys.argv[5]
    y_label = sys.argv[6]
    z_label = sys.argv[7]
    title = sys.argv[8]
    output_image = sys.argv[9]
    section_col = int(sys.argv[10]) if len(sys.argv) == 11 else None
    generate_plot(excel_path, y_col, x_col, z_col, x_label, y_label, z_label, title, output_image, section_col)


if __name__ == "__main__":
    main()