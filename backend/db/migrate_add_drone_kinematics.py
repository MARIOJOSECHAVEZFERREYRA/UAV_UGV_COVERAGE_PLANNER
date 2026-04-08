"""
One-time migration: add horizontal kinematics columns to drones table.

These columns support the semiempirical three-phase (acc/cruise/dec) extension
of the nonlinear energy model.  They are NOT official manufacturer specifications.

Run once:
    source venv/bin/activate
    python -m backend.db.migrate_add_drone_kinematics
"""

from sqlalchemy import text

from backend.database import engine

DEFAULTS = {
    "accel_horizontal_ms2": 1.5,
    "decel_horizontal_ms2": 1.5,
    "power_accel_factor": 1.15,
    "power_decel_factor": 1.05,
}


def run():
    with engine.connect() as conn:
        result = conn.execute(text("PRAGMA table_info(drones)"))
        existing_cols = {row[1] for row in result.fetchall()}

        for col, default in DEFAULTS.items():
            if col not in existing_cols:
                conn.execute(text(
                    f"ALTER TABLE drones ADD COLUMN {col} REAL NOT NULL DEFAULT {default}"
                ))
                print(f"Added column {col} (default {default})")
            else:
                print(f"Column {col} already exists - skipping.")

        conn.commit()

    print("Migration complete.")


if __name__ == "__main__":
    run()
