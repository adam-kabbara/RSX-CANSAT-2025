# UTC Time, Latitude, Longitude, Number of Satellites, Altitude
# GNRMC, GNVTG, GNGGA, GNGLL, GLGSV, GNGSA, GNGGN
# --> Parsing only GNGGA data

import os
from pathlib import Path

def parse_nmea_sentence(sentence):
    """Parse a single NMEA sentence into a dictionary."""
    sentence_type = sentence[1:6]
    data = sentence.split(',')

    parsed_data = {"type": sentence_type}

    if sentence_type == "GNGGA":
        # Parse and format time into HH:MM:SS
        raw_time = data[1]
        formatted_time = (
            f"{raw_time[0:2]}:{raw_time[2:4]}:{raw_time[4:6]}"
            if len(raw_time) >= 6
            else "N/A"
        )

        # Convert latitude to decimal degrees
        raw_lat = data[2]
        lat_direction = data[3]
        latitude = convert_to_decimal(raw_lat, lat_direction)

        # Convert longitude to decimal degrees
        raw_lon = data[4]
        lon_direction = data[5]
        longitude = convert_to_decimal(raw_lon, lon_direction)

        parsed_data.update({
            "time": formatted_time,
            "latitude": latitude,
            "latitude_direction": lat_direction,  # Include N/S
            "longitude": longitude,
            "longitude_direction": lon_direction,  # Include E/W
            "num_satellites": data[7],
            "altitude": data[9]
        })

    return parsed_data

def convert_to_decimal(coordinate, direction):
    """Convert a coordinate from DMM to Decimal Degrees."""
    if not coordinate or len(coordinate) < 4:  # Ensure valid input
        return "N/A"
    
    # Separate degrees and minutes
    if direction in ["N", "S"]:
        degrees = int(coordinate[:2])
        minutes = float(coordinate[2:])
    elif direction in ["E", "W"]:
        degrees = int(coordinate[:3])
        minutes = float(coordinate[3:])
    else:
        return "N/A"

    # Decimal Degrees = Degrees + (Minutes / 60)
    decimal_degrees = degrees + (minutes / 60)

    # Apply negative sign for South or West
    if direction in ["S", "W"]:
        decimal_degrees *= -1

    return round(decimal_degrees, 6)

def format_parsed_data(parsed_data):
    """Format parsed data into the desired output format."""
    formatted_output = ""
    for entry in parsed_data:
        sentence_type = entry["type"]

        # Only print relevant information based on the type
        if sentence_type == "GNGGA":
            formatted_output += f"---------------{sentence_type}---------------\n"
            formatted_output += f"Time: {entry.get('time', 'N/A')} (UTC Time)\n"
            formatted_output += f"Latitude: {entry.get('latitude', 'N/A')}°{entry.get('latitude_direction', '')}\n"
            formatted_output += f"Longitude: {entry.get('longitude', 'N/A')}°{entry.get('longitude_direction', '')}\n"
            formatted_output += f"Satellites: {entry.get('num_satellites', 'N/A')}\n"
            formatted_output += f"Altitude: {entry.get('altitude', 'N/A')}m\n"
            formatted_output += "\n"  # Add a newline after each sentence
    
    return formatted_output

def parse_nmea_data(data):
    """Parse a block of NMEA data."""
    lines = data.strip().split('\n')
    parsed_sentences = []
    for line in lines:
        if line.startswith('$'):
            try:
                parsed_sentence = parse_nmea_sentence(line)
                parsed_sentences.append(parsed_sentence)
            except Exception as e:
                print(f"Failed to parse line: {line} - {e}")
    return parsed_sentences

def main():
    # Get user input for file path
    file_path = input("Enter the file path to the GPS data: ")
    # Get user input for file name to save
    save_filename = input("Enter the name for the saved file (leave blank for 'gps.txt'): ")
    
    # Set default filename if none provided
    if not save_filename:
        save_filename = "gps.txt"

    # Ensure the file is saved to Downloads folder
    downloads_folder = str(Path.home() / "Desktop/RSX-CANSAT-2T4-2T5/RSX-CANSAT-2025/sensor_testing/drone_testing/parsed_data")
    save_path = os.path.join(downloads_folder, save_filename)

    with open(file_path, 'r') as f:
        data = f.read()

    parsed_data = parse_nmea_data(data)
    formatted_output = format_parsed_data(parsed_data)

    # Save formatted output to the specified file in Downloads folder
    with open(save_path, 'w') as output_file:
        output_file.write(formatted_output)

    print(f"Parsed data saved to: {save_path}")

if __name__ == "__main__":
    main()
