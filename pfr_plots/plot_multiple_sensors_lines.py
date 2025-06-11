import pandas as pd
import matplotlib.pyplot as plt
import sys


def generate_plot(excel_path, x_col, y_cols, x_label, y_labels, title, output_image, section_col=None):
    # Read the Excel file
    df = pd.read_excel(excel_path)
    # Get column names by index
    x_col_name = df.columns[x_col]
    y_col_names = [df.columns[y] for y in y_cols]
    section_col_name = df.columns[section_col] if section_col is not None else None
    # Convert x column if it contains datetime.time objects
    if df[x_col_name].apply(lambda v: hasattr(v, 'hour') and hasattr(v, 'minute') and hasattr(v, 'second')).all():
        df[x_col_name] = df[x_col_name].apply(lambda t: t.hour*3600 + t.minute*60 + t.second + getattr(t, 'microsecond', 0)/1e6 if pd.notnull(t) else None)
        x_label = x_label + ' (seconds)'
    elif pd.api.types.is_datetime64_any_dtype(df[x_col_name]):
        df[x_col_name] = pd.to_datetime(df[x_col_name])
    else:
        try:
            df[x_col_name] = pd.to_numeric(df[x_col_name])
        except Exception:
            df[x_col_name] = df[x_col_name].astype(str)
    # Remove rows with missing or invalid x/y values
    cols_to_keep = [x_col_name] + y_col_names + ([section_col_name] if section_col_name else [])
    df = df[cols_to_keep].dropna()
    plt.figure(figsize=(8, 5))
    # Plot all three y columns as separate lines
    for i, y_col_name in enumerate(y_col_names):
        plt.plot(df[x_col_name], df[y_col_name], marker='o', label=y_labels[i] if y_labels and i < len(y_labels) else y_col_name)
    # Background color by section (optional, unchanged)
    if section_col_name:
        unique_sections = df[section_col_name].unique()
        colors = plt.cm.Pastel1.colors if len(unique_sections) <= 9 else plt.cm.tab20.colors
        for i, section in enumerate(unique_sections):
            section_mask = df[section_col_name] == section
            x_vals = df[x_col_name][section_mask]
            if not x_vals.empty:
                if len(x_vals) == 1:
                    idx = x_vals.index[0]
                    try:
                        x0 = float(x_vals.iloc[0])
                        prev_x = float(df[x_col_name].iloc[idx-1]) if idx > 0 else x0 - 1
                        next_x = float(df[x_col_name].iloc[idx+1]) if idx < len(df[x_col_name]) - 1 else x0 + 1
                        left = (x0 + prev_x) / 2 if idx > 0 else x0 - (next_x - x0) / 2
                        right = (x0 + next_x) / 2 if idx < len(df[x_col_name]) - 1 else x0 + (x0 - prev_x) / 2
                        plt.axvspan(left, right, color=colors[i % len(colors)], alpha=1, label=str(section))
                    except Exception:
                        plt.axvspan(x_vals.iloc[0], x_vals.iloc[0], color=colors[i % len(colors)], alpha=1, label=str(section))
                else:
                    left = x_vals.iloc[0]
                    right = x_vals.iloc[-1]
                    try:
                        left = float(left) - 0.5 * abs(float(right) - float(left)) / max(len(x_vals)-1, 1)
                        right = float(right) + 0.5 * abs(float(right) - float(left)) / max(len(x_vals)-1, 1)
                    except Exception:
                        pass
                    plt.axvspan(left, right, color=colors[i % len(colors)], alpha=0.6, label=str(section))
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys(), title=section_col_name, loc='upper right', bbox_to_anchor=(1.15, 1))
    else:
        plt.legend()
    plt.xlabel(x_label)
    plt.ylabel(', '.join(y_labels) if y_labels else ', '.join(y_col_names))
    plt.title(title)
    plt.grid(True)
    plt.xticks(
        ticks=df[x_col_name].iloc[::5],
        labels=df[x_col_name].iloc[::5],
        rotation=45
    )
    plt.tight_layout()
    plt.savefig(output_image)
    print(f"Plot saved as {output_image}")
    plt.show()


def main():
    if len(sys.argv) not in (12, 13):
        print("Usage: python generate_sensor_plot.py <excel_path> <x_col> <y_col1> <y_col2> <y_col3> <x_label> <y_label1> <y_label2> <y_label3> <title> <output_image> [section_col]")
        print("Example: python generate_sensor_plot.py data.xlsx 0 1 2 3 'Time' 'Temp' 'Pressure' 'Humidity' 'Sensor Data' output.png [section_col]")
        sys.exit(1)
    excel_path = sys.argv[1]
    x_col = int(sys.argv[2])
    y_cols = [int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])]
    x_label = sys.argv[6]
    y_labels = [sys.argv[7], sys.argv[8], sys.argv[9]]
    title = sys.argv[10]
    output_image = sys.argv[11]
    section_col = int(sys.argv[12]) if len(sys.argv) == 13 else None
    generate_plot(excel_path, x_col, y_cols, x_label, y_labels, title, output_image, section_col)


if __name__ == "__main__":
    main()