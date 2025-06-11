import pandas as pd
import matplotlib.pyplot as plt
import sys


def generate_plot(excel_path, x_col, y_col, x_label, y_label, title, output_image, section_col=None):
    # Read the Excel file
    df = pd.read_excel(excel_path)
    # Get column names by index
    x_col_name = df.columns[x_col]
    y_col_name = df.columns[y_col]
    section_col_name = df.columns[section_col] if section_col is not None else None
    # Convert x column if it contains datetime.time objects
    if df[x_col_name].apply(lambda v: hasattr(v, 'hour') and hasattr(v, 'minute') and hasattr(v, 'second')).all():
        # Convert datetime.time to seconds since midnight
        df[x_col_name] = df[x_col_name].apply(lambda t: t.hour*3600 + t.minute*60 + t.second + getattr(t, 'microsecond', 0)/1e6 if pd.notnull(t) else None)
        x_label = x_label + ' (seconds)'
    elif pd.api.types.is_datetime64_any_dtype(df[x_col_name]):
        # Convert pandas datetime to matplotlib date format
        df[x_col_name] = pd.to_datetime(df[x_col_name])
    else:
        # Try to convert to numeric if possible
        try:
            df[x_col_name] = pd.to_numeric(df[x_col_name])
        except Exception:
            # As a last resort, convert to string
            df[x_col_name] = df[x_col_name].astype(str)
    # Remove rows with missing or invalid x/y values
    df = df[[x_col_name, y_col_name] + ([section_col_name] if section_col_name else [])].dropna()
    plt.figure(figsize=(8, 5))
    # Plot all points
    plt.plot(df[x_col_name], df[y_col_name], marker='o')
    # Background color by section
    if section_col_name:
        unique_sections = df[section_col_name].unique()
        colors = plt.cm.Pastel1.colors if len(unique_sections) <= 9 else plt.cm.tab20.colors
        for i, section in enumerate(unique_sections):
            section_mask = df[section_col_name] == section
            x_vals = df[x_col_name][section_mask]
            if not x_vals.empty:
                # If only one data point, make the highlight span the width of one point
                if len(x_vals) == 1:
                    idx = x_vals.index[0]
                    try:
                        x0 = float(x_vals.iloc[0])
                        prev_x = float(df[x_col_name].iloc[idx-1]) if idx > 0 else x0 - 1
                        next_x = float(df[x_col_name].iloc[idx+1]) if idx < len(df[x_col_name]) - 1 else x0 + 1
                        left = (x0 + prev_x) / 2 if idx > 0 else x0 - (next_x - x0) / 2
                        right = (x0 + next_x) / 2 if idx < len(df[x_col_name]) - 1 else x0 + (x0 - prev_x) / 2
                        plt.axvspan(left, right, color=colors[i % len(colors)], alpha=1, label=str(section))
                        # Add a black text label at the data point
                        # plt.text(x0, df[y_col_name].iloc[idx], f"{section}", color='black', fontsize=10, ha='center', va='bottom', fontweight='bold')
                    except Exception:
                        plt.axvspan(x_vals.iloc[0], x_vals.iloc[0], color=colors[i % len(colors)], alpha=1, label=str(section))
                        # plt.text(x_vals.iloc[0], df[y_col_name].iloc[idx], f"{section}", color='black', fontsize=10, ha='center', va='bottom', fontweight='bold')
                else:
                    # Make multi-point highlights thicker by expanding a bit on both sides
                    left = x_vals.iloc[0]
                    right = x_vals.iloc[-1]
                    try:
                        left = float(left) - 0.5 * abs(float(right) - float(left)) / max(len(x_vals)-1, 1)
                        right = float(right) + 0.5 * abs(float(right) - float(left)) / max(len(x_vals)-1, 1)
                    except Exception:
                        pass
                    plt.axvspan(left, right, color=colors[i % len(colors)], alpha=0.6, label=str(section))
        # Show legend for colored sections only once
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys(), title=section_col_name, loc='upper right', bbox_to_anchor=(1.15, 1))
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)
    plt.grid(True)
    # Show x-tick labels for every 5th point only
    plt.xticks(
        ticks=df[x_col_name].iloc[::5],
        labels=df[x_col_name].iloc[::5],
        rotation=45
    )
    # Ensure the origin (0,0) is visible
    # plt.xlim(left=min(0, df[x_col_name].min()))
    # plt.ylim(bottom=min(0, df[y_col_name].min()), top=10)
    
    plt.tight_layout()
    plt.savefig(output_image)
    print(f"Plot saved as {output_image}")
    plt.show()  # Optionally display interactively


def main():
    if len(sys.argv) not in (8, 9):
        print("Usage: python generate_sensor_plot.py <excel_path> <x_col> <y_col> <x_label> <y_label> <title> <output_image> [section_col]")
        sys.exit(1)
    excel_path = sys.argv[1]
    x_col = int(sys.argv[2])
    y_col = int(sys.argv[3])
    x_label = sys.argv[4]
    y_label = sys.argv[5]
    title = sys.argv[6]
    output_image = sys.argv[7]
    section_col = int(sys.argv[8]) if len(sys.argv) == 9 else None
    generate_plot(excel_path, x_col, y_col, x_label, y_label, title, output_image, section_col)


if __name__ == "__main__":
    main()