# UTC Time, Latitude, Longitude, Number of Satellites, Altitude
# GNRMC, GNVTG, GNGGA, GNGLL, GLGSV, GNGSA, GNGGN
# --> Parsing only all GPS data

import os
from pathlib import Path

def parse_nmea_sentence(sentence):
    """Parse a single NMEA sentence into a dictionary."""
    sentence_type = sentence[1:6]
    data = sentence.split(',')

    parsed_data = {"type": sentence_type}

    if sentence_type == "GNRMC":
        parsed_data.update({
            "date": data[9],  # Date in DDMMYY format
            "time": data[1],  # Time in HHMMSS format
            "latitude": f"{data[3]} {data[4]}",
            "longitude": f"{data[5]} {data[6]}",
            "speed_knots": data[7],
            "speed_kph": str(float(data[7]) * 1.852),  # Convert knots to km/h
            "satellites": data[8],
            "altitude": "N/A"
        })
    elif sentence_type == "GNVTG":
        parsed_data.update({
            "speed_knots": data[5],
            "speed_kph": str(float(data[7]) * 1.852),  # Convert knots to km/h
            "course_over_ground": data[1],
            "satellites": "N/A",
            "altitude": "N/A"
        })
    elif sentence_type == "GNGGA":
        parsed_data.update({
            "time": data[1],
            "latitude": f"{data[2]} {data[3]}",
            "longitude": f"{data[4]} {data[5]}",
            "fix_quality": data[6],
            "num_satellites": data[7],
            "altitude": f"{data[9]} {data[10]}"
        })
    elif sentence_type == "GNGLL":
        parsed_data.update({
            "latitude": f"{data[1]} {data[2]}",
            "longitude": f"{data[3]} {data[4]}",
            "time": data[5],
            "status": data[6],
            "satellites": "N/A",
            "altitude": "N/A"
        })
    elif sentence_type == "GLGSV" or sentence_type == "GPGSV":
        # Satellite data can be more variable, ensure there are enough elements
        if len(data) > 1:
            parsed_data.update({
                "satellite_count": data[1],
                "satellite_details": data[2:],  # The details of the satellites can vary
                "latitude": "N/A",
                "longitude": "N/A",
                "speed_knots": "N/A",
                "altitude": "N/A"
            })
    elif sentence_type == "GNGSA":
        # Ensure there are enough elements before accessing
        if len(data) >= 15:
            parsed_data.update({
                "mode": data[1],
                "fix_type": data[2],
                "satellites_used": data[3:15],
                "pdop": data[15],
                "hdop": data[16],
                "vdop": data[17],
                "latitude": "N/A",
                "longitude": "N/A",
                "speed_knots": "N/A",
                "altitude": "N/A"
            })
        else:
            # If the GNGSA sentence is incomplete, handle it gracefully
            parsed_data.update({
                "mode": "N/A",
                "fix_type": "N/A",
                "satellites_used": "N/A",
                "pdop": "N/A",
                "hdop": "N/A",
                "vdop": "N/A",
                "latitude": "N/A",
                "longitude": "N/A",
                "speed_knots": "N/A",
                "altitude": "N/A"
            })
    elif sentence_type == "GNGGN":
        # Ensure there are enough elements before accessing
        if len(data) >= 7:
            parsed_data.update({
                "fix_status": data[1],
                "satellites_in_use": data[2],
                "horizontal_dilution_of_precision": data[3],
                "vertical_dilution_of_precision": data[4],
                "position_dilution_of_precision": data[5],
                "satellites_in_view": data[6],
                "latitude": "N/A",
                "longitude": "N/A",
                "speed_knots": "N/A",
                "altitude": "N/A"
            })
        else:
            # If the GNGGN sentence is incomplete, handle it gracefully
            parsed_data.update({
                "fix_status": "N/A",
                "satellites_in_use": "N/A",
                "horizontal_dilution_of_precision": "N/A",
                "vertical_dilution_of_precision": "N/A",
                "position_dilution_of_precision": "N/A",
                "satellites_in_view": "N/A",
                "latitude": "N/A",
                "longitude": "N/A",
                "speed_knots": "N/A",
                "altitude": "N/A"
            })

    return parsed_data

def format_parsed_data(parsed_data):
    """Format parsed data into the desired output format."""
    formatted_output = ""
    for entry in parsed_data:
        sentence_type = entry["type"]
        formatted_output += f"--------------{sentence_type}------------------\n"
        
        # Only print relevant information based on the type
        if sentence_type == "GNRMC":
            formatted_output += f"Date: {entry.get('date', 'N/A')}\n"
            formatted_output += f"Time: {entry.get('time', 'N/A')}\n"
            formatted_output += f"Latitude: {entry.get('latitude', 'N/A')}\n"
            formatted_output += f"Longitude: {entry.get('longitude', 'N/A')}\n"
            formatted_output += f"Speed: {entry.get('speed_knots', 'N/A')} knots ({entry.get('speed_kph', 'N/A')} km/h)\n"
            formatted_output += f"Satellites: {entry.get('satellites', 'N/A')}\n"
            formatted_output += f"Altitude: {entry.get('altitude', 'N/A')}\n"
        elif sentence_type == "GNVTG":
            formatted_output += f"Speed: {entry.get('speed_knots', 'N/A')} knots ({entry.get('speed_kph', 'N/A')} km/h)\n"
            formatted_output += f"Course: {entry.get('course_over_ground', 'N/A')}\n"
        elif sentence_type == "GNGGA":
            formatted_output += f"Time: {entry.get('time', 'N/A')}\n"
            formatted_output += f"Latitude: {entry.get('latitude', 'N/A')}\n"
            formatted_output += f"Longitude: {entry.get('longitude', 'N/A')}\n"
            formatted_output += f"Fix Quality: {entry.get('fix_quality', 'N/A')}\n"
            formatted_output += f"Satellites: {entry.get('num_satellites', 'N/A')}\n"
            formatted_output += f"Altitude: {entry.get('altitude', 'N/A')}\n"
        elif sentence_type == "GNGLL":
            formatted_output += f"Latitude: {entry.get('latitude', 'N/A')}\n"
            formatted_output += f"Longitude: {entry.get('longitude', 'N/A')}\n"
            formatted_output += f"Time: {entry.get('time', 'N/A')}\n"
            formatted_output += f"Status: {entry.get('status', 'N/A')}\n"
        elif sentence_type == "GLGSV" or sentence_type == "GPGSV":
            formatted_output += f"Satellite Count: {entry.get('satellite_count', 'N/A')}\n"
            formatted_output += f"Satellite Details: {', '.join(entry.get('satellite_details', []))}\n"
        elif sentence_type == "GNGSA":
            formatted_output += f"Mode: {entry.get('mode', 'N/A')}\n"
            formatted_output += f"Fix Type: {entry.get('fix_type', 'N/A')}\n"
            formatted_output += f"Satellites Used: {', '.join(entry.get('satellites_used', []))}\n"
            formatted_output += f"PDOP: {entry.get('pdop', 'N/A')}\n"
            formatted_output += f"HDOP: {entry.get('hdop', 'N/A')}\n"
            formatted_output += f"VDOP: {entry.get('vdop', 'N/A')}\n"
        elif sentence_type == "GNGGN":
            formatted_output += f"Fix Status: {entry.get('fix_status', 'N/A')}\n"
            formatted_output += f"Satellites In Use: {entry.get('satellites_in_use', 'N/A')}\n"
            formatted_output += f"HDOP: {entry.get('horizontal_dilution_of_precision', 'N/A')}\n"
            formatted_output += f"VDOP: {entry.get('vertical_dilution_of_precision', 'N/A')}\n"
            formatted_output += f"PDOP: {entry.get('position_dilution_of_precision', 'N/A')}\n"
            formatted_output += f"Satellites In View: {entry.get('satellites_in_view', 'N/A')}\n"

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
    downloads_folder = str(Path.home() / "Downloads")
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
