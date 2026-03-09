import math
import json
from shapely.geometry import Point


class GeoUtils:
    """
    Utilities for converting flat coordinates to geodetic (WGS84)
    and exporting missions.
    Default reference: Santa Cruz, Bolivia.
    """
    
    # WGS84 Constants
    R_EARTH = 6378137.0

    @staticmethod
    def enu_to_geodetic(x, y, z, lat0, lon0, alt0):
        """
        Converts local ENU (East-North-Up) coordinates to GPS (Lat, Lon, Alt).
        """
        rad_lat0 = math.radians(lat0)
        
        # Differences in meters to degrees
        d_lat = y / GeoUtils.R_EARTH
        d_lon = x / (GeoUtils.R_EARTH * math.cos(rad_lat0))

        new_lat = lat0 + math.degrees(d_lat)
        new_lon = lon0 + math.degrees(d_lon)
        new_alt = alt0 + z

        return new_lat, new_lon, new_alt

    @staticmethod
    def export_qgc_mission(waypoints, filename, home_lat=-17.3935, home_lon=-63.2622):
        """
        Generates a .plan file compatible with QGroundControl / PX4.
        :param waypoints: List of tuples (x, y) in meters.
        :param filename: Output filename.
        :param home_lat: Latitude of the local (0,0) point.
        :param home_lon: Longitude of the local (0,0) point.
        """
        mission_items = []
        altitude = 10.0 # Flight altitude AGL (Above Ground Level) in meters
        
        # 1. Initial Waypoint (Takeoff)
        mission_items.append(GeoUtils._create_mission_item(0, home_lat, home_lon, 0, "TAKEOFF"))

        # 2. Spraying route
        for i, (x, y) in enumerate(waypoints):
            # Convert each point (x,y) meters -> Global Lat/Lon
            lat, lon, alt = GeoUtils.enu_to_geodetic(x, y, altitude, home_lat, home_lon, 0)
            mission_items.append(GeoUtils._create_mission_item(i+1, lat, lon, alt, "WAYPOINT"))

        # 3. QGroundControl JSON Structure
        plan = {
            "fileType": "Plan",
            "geoFence": {"circles": [], "polygons": [], "version": 2},
            "groundStation": "AgriSwarm Planner",
            "mission": {
                "cruiseSpeed": 5,
                "firmwareType": 12, # PX4
                "hoverSpeed": 0,
                "items": mission_items,
                "plannedHomePosition": [home_lat, home_lon, 0],
                "vehicleType": 2, # Multirotor
                "version": 2
            },
            "rallyPoints": {"points": [], "version": 2},
            "version": 1
        }

        with open(filename, 'w') as f:
            json.dump(plan, f, indent=4)
        
        print(f"Mission exported (Ref: Santa Cruz): {filename}")

    @staticmethod
    def snap_points_to_polygon_offset(polygon, points, offset):
        """
        Snap a list of (x, y) points onto the exterior ring of a buffered polygon.

        :param polygon: Shapely Polygon (field boundary).
        :param points: List of (x, y) tuples to snap.
        :param offset: Buffer distance from the polygon boundary.
        :return: List of snapped (x, y) tuples, or None on failure.
        """
        try:
            offset_poly = polygon.buffer(offset, join_style=2)
            if offset_poly.is_empty:
                return None

            if offset_poly.geom_type == 'MultiPolygon':
                offset_poly = max(offset_poly.geoms, key=lambda p: p.area)

            target_ring = offset_poly.exterior
            snapped = []
            for p in points:
                pt = Point(p)
                d = target_ring.project(pt)
                new_pt = target_ring.interpolate(d)
                snapped.append((new_pt.x, new_pt.y))
            return snapped
        except Exception:
            return None

    @staticmethod
    def _create_mission_item(seq, lat, lon, alt, type_cmd):
        """Helper to create MAVLink mission items"""
        # 22 = TAKEOFF, 16 = WAYPOINT
        cmd_id = 22 if type_cmd == "TAKEOFF" else 16 
        return {
            "autoContinue": True,
            "command": cmd_id,
            "doJumpId": seq,
            "frame": 3, # Global Relative Altitude (3)
            "params": [0, 0, 0, 0, lat, lon, alt],
            "type": "SimpleItem"
        }