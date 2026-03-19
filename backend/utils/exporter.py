import json

class MissionExporter:
    """
    Exports routes to the standard QGroundControl/PX4 .plan format.
    """

    @staticmethod
    def save_plan(filename, waypoints_latlon):
        """
        :param filename: Name of the .plan file
        :param waypoints_latlon: List of tuples [(lat, lon, alt), ...]
        """
        mission_items = []
        
        # 1. Configure the first item (Takeoff or Dummy)
        # For simplicity, we assume the list contains flight waypoints
        for i, (lat, lon, alt) in enumerate(waypoints_latlon):
            item = {
                "autoContinue": True,
                "command": 16,  # MAV_CMD_NAV_WAYPOINT
                "doJumpId": i + 1,
                "frame": 3,     # MAV_FRAME_GLOBAL_RELATIVE_ALT
                "params": [0, 0, 0, 0, lat, lon, alt],
                "type": "SimpleItem"
            }
            mission_items.append(item)

        # 2. Complete JSON structure required by QGC
        plan_dict = {
            "fileType": "Plan",
            "geoFence": {"circles": [], "polygons": [], "version": 2},
            "groundStation": "AgriSwarm Planner",
            "mission": {
                "cruiseSpeed": 5, # m/s
                "firmwareType": 12, # PX4
                "hoverSpeed": 0,
                "items": mission_items,
                "plannedHomePosition": [waypoints_latlon[0][0], waypoints_latlon[0][1], 0],
                "vehicleType": 2, # Multirotor
                "version": 2
            },
            "rallyPoints": {"points": [], "version": 2},
            "version": 1
        }

        # 3. Save file
        with open(filename, 'w') as f:
            json.dump(plan_dict, f, indent=4)
        print(f"Mission exported successfully: {filename}")