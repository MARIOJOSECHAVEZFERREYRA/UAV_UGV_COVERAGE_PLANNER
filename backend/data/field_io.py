import json
import os
from shapely.geometry import Polygon

class FieldIO:
    """
    Module to save and load agricultural fields in simple JSON format.
    """

    @staticmethod
    def save_field(polygon: Polygon, filename: str):
        """Saves the polygon coordinates to a JSON file."""
        # Ensure the directory exists
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        # Extract coordinates as a list of tuples
        coords = list(polygon.exterior.coords)
        
        data = {
            "type": "Polygon",
            "coordinates": coords
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Field saved to: {filename}")

    @staticmethod
    def load_field(filename: str):
        """
        Loads a polygon from a JSON file.
        Supports Legacy format: {"coordinates": [...]}
        Supports New format:    {"boundary": [...], "obstacles": [[[x,y],...]]}
        
        Returns:
            tuple: (shapely.geometry.Polygon, list_of_list_of_points_obstacles)
        """
        if not os.path.exists(filename):
            raise FileNotFoundError(f"File not found: {filename}")
            
        with open(filename, 'r') as f:
            data = json.load(f)
            
        # Determine format
        if "boundary" in data:
            # New Format
            coords = data["boundary"]
            obstacles = data.get("obstacles", [])
        elif "coordinates" in data:
            # Legacy Format (No obstacles support)
            coords = data["coordinates"]
            obstacles = []
        else:
            raise KeyError("Invalid JSON: Must contain 'boundary' or 'coordinates'")
            
        return Polygon(coords), obstacles