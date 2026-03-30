"""Interactive drone selector for tests. Reads from DroneDB."""

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from data import DroneDB


def pick_drone() -> str:
    """Show available drones and let the user pick one. Returns drone name."""
    names = DroneDB.get_drone_names()
    if not names:
        print("No drones in database.")
        sys.exit(1)

    print("\nAvailable drones:")
    for i, name in enumerate(names):
        spec = DroneDB.get_specs(name)
        swath_str = ""
        if spec.spray and spec.spray.swath_m:
            lo = float(spec.spray.swath_m[0].value)
            hi = float(spec.spray.swath_m[1].value)
            swath_str = f"  swath {lo}-{hi}m"
        print(f"  [{i+1}] {name}{swath_str}")

    choice = input("\nEnter number [default=1]: ").strip()
    if not choice:
        return names[0]
    if choice.isdigit() and 1 <= int(choice) <= len(names):
        return names[int(choice) - 1]
    return names[0]


def get_swath_from_drone(drone_name: str) -> float:
    """Extract max swath width from drone specs."""
    spec = DroneDB.get_specs(drone_name)
    if spec and spec.spray and spec.spray.swath_m:
        return float(spec.spray.swath_m[1].value)
    return 5.0
