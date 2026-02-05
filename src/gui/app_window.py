import sys
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QComboBox, QPushButton, QTextEdit, QMessageBox, 
                             QFrame, QSizePolicy, QFormLayout, QDoubleSpinBox, QCheckBox, 
                             QStackedWidget, QFileDialog)
from PyQt6.QtCore import Qt
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import substring
import math
import datetime

# Local Imports
from data import DroneDB, SpecValue
from data.field_io import FieldIO
from algorithms.analysis import MissionAnalyzer
from utils import GeoUtils

from gui.map_widget import MapWidget
from gui.report_panel import ReportPanel
from gui.ui_builder import UIBuilder
from gui.styles import *
from controllers.mission_controller import MissionController

class AgriSwarmApp(QMainWindow):
    def __init__(self, filename=None):
        super().__init__()
        self.filename = filename
        self.setWindowTitle("AgriSwarm Planner - Thesis v2.5")
        self.resize(1200, 800)
        # Apply only global styles (Dialogs, etc). Sidebar has its own.
        self.setStyleSheet(QMESSAGEBOX_STYLE)

        # Logical State
        self.points = []
        self.polygon = None
        self.current_drone = "DJI Agras T30"
        self.controller = MissionController()
        self.best_path = None
        self.metrics = None
        self.truck_dist = 0
        self.last_mission_cycles = None # Mobile by default
        self.static_cycles = None
        self.current_specs = None
        self.safe_polygon = None



        # --- MAIN LAYOUT ---
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 1. MAP (Left)
        self.map_widget = MapWidget(self)
        layout.addWidget(self.map_widget, stretch=1)

        # Connect Signals
        # Use UniqueConnection to prevent duplicate signal handling (Fixes Right-Click X3 bug)
        self.map_widget.map_clicked.connect(self.on_map_left_click, Qt.ConnectionType.UniqueConnection)
        self.map_widget.map_right_clicked.connect(self.on_map_right_click, Qt.ConnectionType.UniqueConnection)
        self.map_widget.point_moved.connect(self.on_point_moved, Qt.ConnectionType.UniqueConnection)

        # 2. SIDEBAR (Right) - Stacked (Control Panel / Report)
        self.sidebar_stack = QStackedWidget()
        self.sidebar_stack.setFixedWidth(380)
        layout.addWidget(self.sidebar_stack)

        # --- PANEL 0: CONTROLS ---
        self.control_panel = QWidget()
        self.control_panel.setStyleSheet(SIDEBAR_STYLE)
        self.sidebar_stack.addWidget(self.control_panel)

        side_layout = QVBoxLayout(self.control_panel)
        side_layout.setContentsMargins(20, 25, 20, 25)
        side_layout.setSpacing(15)

        # --- SIDEBAR CONTENT ---
        UIBuilder.create_sidebar_header(side_layout)
        
        self.combo_drones = UIBuilder.create_drone_selector(side_layout, self.on_drone_changed)
        
        # --- MISSION PARAMETERS (Overrides) ---
        param_widgets = UIBuilder.create_mission_parameters(side_layout)
        self.spin_swath = param_widgets['swath']
        self.spin_tank = param_widgets['tank']
        self.spin_speed = param_widgets['speed']
        self.spin_app_rate = param_widgets['app_rate']
        self.spin_truck_offset = param_widgets['truck_offset']
        self.spin_truck_offset.valueChanged.connect(self.on_truck_offset_changed)



        
        self.btn_clear, self.btn_load = UIBuilder.create_geometry_buttons(
            side_layout, self.clear_canvas, self.load_field
        )
        
        # Custom Route Button
        self.btn_draw_route = UIBuilder.create_custom_route_button(side_layout, self.toggle_draw_route)
        
        # Obstacle Button
        self.btn_draw_obstacle = UIBuilder.create_obstacle_button(side_layout, self.toggle_draw_obstacle)
        
        
        # Visual Options
        self.chk_swath, self.chk_mode_static = UIBuilder.create_visual_options(
            side_layout, self.on_swath_toggled, self.toggle_visualization_mode
        )

        # Algorithm Selection (Thesis Feature)
        self.chk_algo_formal = UIBuilder.create_algorithm_selector(side_layout, self.on_algo_toggled)

        
        self.btn_calc, self.btn_report_window, self.btn_export = UIBuilder.create_action_buttons(
            side_layout, self.run_optimization, self.show_comparative_report, self.export_mission
        )

        self.map_widget.clear_map()        
        self.on_drone_changed(self.combo_drones.currentText())        
        self.update_ui_state()

    def update_ui_state(self):
        """Enable/Disable controls based on current state (Field Loaded, Drawing, etc)"""
        # A field is available if we have a polygon OR editor points
        has_polygon = self.polygon is not None
        has_points = len(self.points) >= 3  # Minimum for a valid polygon
        has_field = has_polygon or has_points
        
        is_drawing = self.btn_draw_route.isChecked()
        is_obstacle_mode = self.btn_draw_obstacle.isChecked()
        has_results = self.best_path is not None
        
        # 1. Controls dependent on Field
        self.btn_draw_route.setEnabled(has_polygon and not is_obstacle_mode)
        self.btn_draw_obstacle.setEnabled(has_polygon and not is_drawing)
        self.spin_truck_offset.setEnabled(has_polygon)
        
        # Calc needs field (points or polygon) and NOT be drawing
        self.btn_calc.setEnabled(has_field and not is_drawing and not is_obstacle_mode)
        
        # Clear is always available (to reset state)
        self.btn_clear.setEnabled(True)
        
        # Load disabled while drawing
        self.btn_load.setEnabled(not is_drawing and not is_obstacle_mode)
        
        # 2. Controls dependent on Results
        self.btn_export.setEnabled(has_results)
        self.btn_report_window.setEnabled(has_results)
        self.chk_mode_static.setEnabled(has_results)
        
        # 3. Parameters - can be changed anytime without restrictions
        self.spin_tank.setEnabled(True)
        self.spin_speed.setEnabled(True)
        self.spin_app_rate.setEnabled(True)
        self.combo_drones.setEnabled(True)

    def add_separator(self, layout):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Plain)
        line.setStyleSheet("background-color: #455a64; max-height: 1px;")
        layout.addWidget(line)

    # --- LOGIC ---

    def on_map_left_click(self, x, y):
        if self.best_path: return
        
        # Obstacle Mode
        if self.map_widget.draw_mode_obstacle:
            self.map_widget.temp_obstacle_points.append((x, y))
            self.map_widget.draw_obstacles()
            return

        self.points.append((x, y))
        self.map_widget.draw_editor_state(self.points)
        
        # Auto-create polygon if we have enough points
        if len(self.points) >= 3:
            self.polygon = Polygon(self.points)
            self.update_ui_state()

    def on_map_right_click(self, x, y):
        print(f"DEBUG: Right Click at {x}, {y}. Obstacle Mode: {self.map_widget.draw_mode_obstacle}")
        if self.best_path: return
        
        # Obstacle Mode: Right Click = Undo Logic
        if self.map_widget.draw_mode_obstacle:
            if self.map_widget.temp_obstacle_points:
                print("DEBUG: Undoing last obstacle point")
                # Undo last point
                self.map_widget.temp_obstacle_points.pop()
                self.map_widget.draw_obstacles()
                self.statusBar().showMessage("Point Undo", 1000)
            else:
                print("DEBUG: No points to undo")
            return

        if self.points:
            self.points.pop()
            self.map_widget.draw_editor_state(self.points)
            
            # Update polygon or clear it if not enough points
            if len(self.points) >= 3:
                self.polygon = Polygon(self.points)
            else:
                self.polygon = None
            self.update_ui_state()

    def on_point_moved(self, index, x, y):
        if self.best_path: return
        if 0 <= index < len(self.points):
            self.points[index] = (x, y)
            self.map_widget.draw_editor_state(self.points)
            
            # Update polygon
            if len(self.points) >= 3:
                self.polygon = Polygon(self.points)
                self.update_ui_state()

    def clear_canvas(self):
        self.points = []
        self.best_path = None
        self.polygon = None
        self.btn_export.setEnabled(False)
        self.map_widget.full_reset()
        self.update_ui_state()

    def on_truck_offset_changed(self, value):
        """Handles interactive change of station distance (snap)"""
        if not self.polygon: return

        # FAST UPDATE: If we have a calculated mission, re-run logistics ONLY
        if self.best_path and not getattr(self, '_is_recalculating', False):
            # Debounce/throttle could be useful, but let's try direct update
            # Set flag to prevent recursion if needed
            self._is_recalculating = True
            try:
                # Re-use the existing logic but pass the existing path
                # Reuse overrides logic from run_optimization
                overrides = {
                    'swath': self.spin_swath.value(),
                    'tank': self.spin_tank.value(),
                    'speed': self.spin_speed.value(),
                    'app_rate': self.spin_app_rate.value(),
                }
                
                # Use current manual service points if available?
                # If we are in "Result Mode", usually we default to auto buffer unless manual route was used.
                # If manual route was used, we should snap it.
                service_points = getattr(self.map_widget, 'service_route_points', [])
                
                # SNAP LOGIC (Copied/Refined)
                # If we have manual points, we snap them first
                if service_points and hasattr(self, 'original_manual_route') and self.original_manual_route:
                     # Calculate NEW snapped points
                     # Reuse existing snap logic below or rely on Controller's internal snap?
                     # Controller snaps manually passed points using truck_offset.
                     # So we can pass the ORIGINAL manual points and let the controller snap them with new offset!
                     points_to_pass = self.original_manual_route
                else:
                     # Auto mode (Controller handles it based on poly)
                     points_to_pass = []
                
                # CALL CONTROLLER (Fast Mode)
                result = self.controller.run_mission_planning(
                    polygon_points=self.points,
                    drone_name=self.current_drone,
                    overrides=overrides,
                    truck_route_points=points_to_pass,
                    truck_offset=value,
                    use_mobile_station=True,
                    precalculated_path=self.best_path # KEY: Reuse path!
                )
                
                # Update State Partial
                self.mission_cycles = result['mission_cycles']
                self.last_mission_cycles = self.mission_cycles
                self.static_cycles = result.get('static_cycles')
                self.current_results = result
                
                # Redraw
                use_static = self.chk_mode_static.isChecked()
                self.map_widget.draw_results(
                    self.polygon, 
                    self.safe_polygon, 
                    self.mission_cycles,
                    use_static,
                    road_geom=result.get('truck_route_line')
                )
                
            except Exception as e:
                print(f"Fast Update Failed: {e}")
            finally:
                self._is_recalculating = False
            return
        
        # MANUAL DRAWING SNAP (Original Logic)
        service_points = getattr(self.map_widget, 'service_route_points', [])
        
        # Ignore if we are drawing
        if hasattr(self.map_widget, 'temp_route_points') and self.map_widget.temp_route_points:
             return

        if not service_points: return

        # Save original if it doesn't exist
        if not hasattr(self, 'original_manual_route') or self.original_manual_route is None:
             self.original_manual_route = list(service_points)
        
        # Reset if offset is 0
        if value < 0.1:
            self.map_widget.service_route_points = list(self.original_manual_route)
            self.map_widget.draw_service_route(False)
            self.map_widget.update()
            return

        # Snap Logic (Visual Only)
        try:
             offset_poly = self.polygon.buffer(value, join_style=2)
             target_ring = None
             
             if not offset_poly.is_empty:
                 if offset_poly.geom_type == 'Polygon':
                     target_ring = offset_poly.exterior
                 elif offset_poly.geom_type == 'MultiPolygon':
                     largest = max(offset_poly.geoms, key=lambda p: p.area)
                     target_ring = largest.exterior
            
             if target_ring:
                 new_points = []
                 for p in self.original_manual_route:
                     pt = Point(p)
                     d = target_ring.project(pt)
                     new_pt = target_ring.interpolate(d)
                     new_points.append((new_pt.x, new_pt.y))
                 
                 self.map_widget.service_route_points = new_points
                 self.map_widget.draw_service_route(False)
                 self.map_widget.update()
                 
        except Exception as e:
             print(f"Error interactive snap: {e}")

    def load_field(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, 
            "Load Field", 
            "", 
            "JSON Files (*.json);;All Files (*)"
        )
        
        if not filename: return

        try:
            self.filename = filename 
            poly = FieldIO.load_field(self.filename)
            
            # FORCE 2D: Strip Z coordinate if present to avoid "too many values to unpack" errors
            # shapely coords can be (x, y, z), but our logic expects (x, y)
            raw_coords = list(poly.exterior.coords)
            self.points = [(p[0], p[1]) for p in raw_coords][:-1]            
            self.polygon = Polygon(self.points) 

            self.best_path = None
            self.map_widget.draw_editor_state(self.points)
            self.setWindowTitle(f"AgriSwarm Planner - {filename.split('/')[-1]}")
            self.update_ui_state()
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def toggle_draw_route(self):
        is_drawing = self.btn_draw_route.isChecked()
        
        # Ensure mutual exclusion
        if is_drawing and self.btn_draw_obstacle.isChecked():
            self.btn_draw_obstacle.click() # Toggle off

        # Connect/Reconnect Signal to ensure update
        try: self.map_widget.route_length_changed.disconnect(self.on_route_length_update)
        except: pass
        self.map_widget.route_length_changed.connect(self.on_route_length_update)
        
        if is_drawing:
             # Refresh map: Clear everything, draw field polygon (closed)
             # This visually "closes" the field and prepares for the new route drawing
             self.map_widget.draw_editor_state(self.points)
             self.original_manual_route = None
             
             # NOW enable route mode
             self.map_widget.set_draw_mode_route(True)
             
             self.btn_draw_route.setText("FINISH ROUTE (0 m)")
             # Orange button for active state
             self.btn_draw_route.setStyleSheet("background-color: #e67e22; color: white; border: none; padding: 10px; font-weight: bold; border-radius: 4px;")
        else:
             self.map_widget.set_draw_mode_route(False)
             self.btn_draw_route.setText("DRAW MOBILE ROUTE")
             # Grey button for inactive
             self.btn_draw_route.setStyleSheet("background-color: #7f8c8d; color: white; border: none; padding: 10px; font-weight: bold; border-radius: 4px;")
        
        # Update UI state based on drawing mode
        self.update_ui_state()

    def toggle_draw_obstacle(self):
        is_active = self.btn_draw_obstacle.isChecked()
        print(f"DEBUG: toggle_draw_obstacle triggered. Active: {is_active}")
        
        # Ensure mutual exclusion
        if is_active and self.btn_draw_route.isChecked():
            self.btn_draw_route.click() # Toggle off
            print("DEBUG: Toggled off route drawing due to exclusion.")
            
        if is_active:
            # Entering Mode
            print("DEBUG: Entering Obstacle Mode")
            self.map_widget.set_draw_mode_obstacle(True)
            self.btn_draw_obstacle.setText("FINISH OBSTACLE")
            # Red button for active state
            self.btn_draw_obstacle.setStyleSheet("background-color: #c0392b; color: white; border: none; padding: 10px; font-weight: bold; border-radius: 4px;")
        else:
            # Exiting Mode (Commit)
            print(f"DEBUG: Exiting Obstacle Mode. Points: {len(self.map_widget.temp_obstacle_points)}")
            if len(self.map_widget.temp_obstacle_points) >= 3:
                # Commit
                new_obs = list(self.map_widget.temp_obstacle_points)
                self.map_widget.obstacle_polygons.append(new_obs)
                self.statusBar().showMessage("Obstacle Added", 3000)
                print("DEBUG: Obstacle Added")
            else:
                if self.map_widget.temp_obstacle_points:
                    self.statusBar().showMessage("Obstacle discarded (too few points)", 3000)
                    print("DEBUG: Obstacle discarded")

            self.map_widget.temp_obstacle_points = []
            
            self.map_widget.set_draw_mode_obstacle(False)
            self.btn_draw_obstacle.setText("ADD OBSTACLE")
            # Grey button for inactive
            self.btn_draw_obstacle.setStyleSheet("background-color: #7f8c8d; color: white; border: none; padding: 10px; font-weight: bold; border-radius: 4px;")
            
            # Redraw
            self.map_widget.draw_editor_state(self.points)
            self.map_widget.draw_obstacles() 
            
        self.update_ui_state()

    def on_route_length_update(self, length):
        """Update button text with dynamic length"""
        if self.btn_draw_route.isChecked():
            self.btn_draw_route.setText(f"FINISH ROUTE ({length:.0f} m)")

    def on_drone_changed(self, drone_name):
        self.current_drone = drone_name

        
        spec = DroneDB.get_specs(drone_name)
        if spec:
            self.current_specs = spec # Store for reports
            # Populate UI with defaults from DB
            
            # Swath Handling (New Smart Logic)
            # DroneDB swath_m is typically (SpecValue(min), SpecValue(max))
            swath_range = spec.spray.swath_m
            
            min_swath = 1.0
            max_swath = 20.0
            default_swath = 5.0
            
            if isinstance(swath_range, tuple) and len(swath_range) == 2:
                # Extract min/max from SpecValue objects
                try:
                    val1 = float(swath_range[0].value)
                    val2 = float(swath_range[1].value)
                    min_swath = min(val1, val2)
                    max_swath = max(val1, val2)
                    default_swath = max_swath # Default to max width for efficiency
                except (ValueError, AttributeError):
                    pass
            elif isinstance(swath_range, SpecValue):
                # Fallback if it's a single value (unlikely based on DroneDB structure but safe)
                val = float(swath_range.value)
                min_swath = val
                max_swath = val
                default_swath = val

            self.spin_swath.setRange(min_swath, max_swath)
            self.spin_swath.setValue(default_swath)
            self.spin_swath.setToolTip(f"Allowed range: {min_swath}m - {max_swath}m")
            
            self.spin_tank.setValue(spec.spray.tank_l.value)            
            self.spin_speed.setValue(spec.flight.work_speed_kmh.value / 3.6) # km/h -> m/s
            
            # Flow
            # Application rate is user-configured, not from specs
            # Keep default 20 L/ha or user's last value
            pass

    def run_optimization(self):
        """
        Calculates the optimal mission using MissionController.
        delegates geometry, planning, and logistics logic.
        """
        # 1. Validate Input
        if len(self.points) < 3: 
            return

        # 2. Collect Overrides from UI
        overrides = {
            'swath': self.spin_swath.value(),
            'tank': self.spin_tank.value(),
            'speed': self.spin_speed.value(),
            'app_rate': self.spin_app_rate.value(),
        }
        
        truck_offset = self.spin_truck_offset.value()
        
        # 3. Get Routes from Map
        temp_points = getattr(self.map_widget, 'temp_route_points', [])
        service_points = getattr(self.map_widget, 'service_route_points', [])
        
        # Check for Unfinished Route
        if len(temp_points) >= 2:
             print("Auto-committing unfinished route...")
             service_points = list(temp_points)
             self.map_widget.service_route_points = service_points
             self.map_widget.temp_route_points = []
             self.map_widget.set_draw_mode_route(False)

        # Validate Custom Route if active
        if self.btn_draw_route.isChecked() and (not service_points or len(service_points) < 2):
             QMessageBox.warning(self, "Empty Route", "'Draw Route' mode is active but there is no valid route.\n\nPlease draw at least 2 points (Left Click) and Finish (Right Click),\nor disable the mode.")
             return

        # 4. Execute Mission Planning via Controller
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        self.statusBar().showMessage("Calculating Mission...")
        QApplication.processEvents()
        
        try:
            # Prepare points list
            poly_points = self.points
            
            # Get Obstacles
            obstacles = self.map_widget.obstacle_polygons
            
            result = self.controller.run_mission_planning(
                polygon_points=poly_points,
                drone_name=self.current_drone,
                overrides=overrides,
                truck_route_points=service_points,
                truck_offset=truck_offset,
                use_mobile_station=True,
                strategy_name="decomposition" if self.chk_algo_formal.isChecked() else "genetic", 
                obstacle_polygons=obstacles # Pass obstacles
            )
            
            # 5. Update State
            self.polygon = result['polygon']
            self.safe_polygon = result['safe_polygon']
            self.mission_cycles = result['mission_cycles']
            
            # Store for Toggles / Export
            self.last_mission_cycles = self.mission_cycles
            self.static_cycles = result.get('static_cycles') # Ensure controller returns this
            self.current_results = result
            self.best_path = result.get('best_path') # Store LineString object!
            
            # 6. Update UI (Visuals)
            is_static = False 
            
            self.map_widget.draw_results(
                self.polygon, 
                self.safe_polygon, 
                self.mission_cycles,
                is_static,
                road_geom=result.get('truck_route_line')
            )
            
            # 7. Update Buttons
            self.update_ui_state()
            self.statusBar().showMessage("Mission Calculated Successfully", 5000)
            
            # Enable Export
            self.btn_export.setEnabled(True)
            self.current_results = result # Store for export
            self.statusBar().showMessage("Mission Calculated Successfully", 5000)
            
        except ValueError as ve:
             QMessageBox.critical(self, "Validation Error", str(ve))
             self.statusBar().clearMessage()
        except Exception as e:
             import traceback
             traceback.print_exc()
             QMessageBox.critical(self, "Error", f"An unexpected error occurred: {str(e)}")
             self.statusBar().clearMessage()
        finally:
            QApplication.restoreOverrideCursor()

    def show_report_panel(self, metrics, comparison, resources):
        # Create Report Panel
        # Clear previous report if any
        if self.sidebar_stack.count() > 1:
            w = self.sidebar_stack.widget(1)
            self.sidebar_stack.removeWidget(w)
            w.deleteLater()
        
        report_panel = ReportPanel(metrics, comparison, resources)
        report_panel.back_clicked.connect(self.show_control_panel)
        self.sidebar_stack.addWidget(report_panel)
        self.sidebar_stack.setCurrentIndex(1)

    def export_mission(self):
        if not self.last_mission_cycles: return
        
        try:
            filename, _ = QFileDialog.getSaveFileName(
                self,
                "Export Mission",
                "mision_agriswarm.json",
                "JSON Files (*.json);;All Files (*)"
            )
            
            if not filename: return
            
            import json
            
            # Helper for non-serializable objects (like numpy arrays)
            def default_serializer(obj):
                if hasattr(obj, 'tolist'): return obj.tolist()
                return str(obj)
                
            # Extract Polygon Coords for Re-loading
            poly_coords = []
            if self.polygon:
                if hasattr(self.polygon, 'exterior'):
                     poly_coords = list(self.polygon.exterior.coords)

            # Structured Export
            data = {
                "type": "AgriSwarmSession",
                "version": "1.0",
                "polygon": poly_coords,
                "mission_cycles": self.last_mission_cycles
            }

            with open(filename, 'w') as f:
                json.dump(data, f, indent=4, default=default_serializer)

            QMessageBox.information(self, "Export", f"Mission saved successfully to:\n{filename}")
            
        except Exception as e:
            QMessageBox.critical(self, "Export Error", str(e))
        
    def show_comparative_report(self):
        """Generates and displays the Thesis report in the sidebar"""
        if not self.last_mission_cycles or not self.current_specs or not self.static_cycles:
            return
            
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            # 1. Use Pre-calculated Static Mission (Baseline)
            static_cycles = self.static_cycles
            
            # 2. Compare
            comparison = MissionAnalyzer.compare_missions(self.last_mission_cycles, static_cycles)
            
            # 3. Logistics Plan (Resources)
            resources = MissionAnalyzer.plan_logistics(self.last_mission_cycles, self.current_specs)
            
            # 4. Comprehensive Metrics (Executive Summary)
            comprehensive = MissionAnalyzer.calculate_comprehensive_metrics(
                self.last_mission_cycles, 
                self.polygon, 
                self.current_specs
            )
           
            QApplication.restoreOverrideCursor()
            
            # 5. Create Report Panel
            # Check if a previous report panel exists at index 1 and delete it
            if self.sidebar_stack.count() > 1:
                old = self.sidebar_stack.widget(1)
                self.sidebar_stack.removeWidget(old)
                old.deleteLater()
                
            report_panel = ReportPanel(comprehensive, comparison, resources)
            report_panel.back_clicked.connect(self.show_control_panel)
            
            self.sidebar_stack.addWidget(report_panel)
            self.sidebar_stack.setCurrentIndex(1)
            
        except Exception as e:
            QApplication.restoreOverrideCursor()
            QMessageBox.critical(self, "Report Error", str(e))
            
    def on_swath_toggled(self, state):
        visible = bool(state)
        # Update widget state
        self.map_widget.show_swath = visible
        
        if self.best_path and (self.last_mission_cycles or self.static_cycles):
             # Mode Mission: Redraw Results
             use_static = self.chk_mode_static.isChecked()
             cycles_to_draw = self.static_cycles if use_static else self.last_mission_cycles
             
             if cycles_to_draw:
                # Pass road_geom if available
                road = getattr(self, 'road_geom', None)
                self.map_widget.draw_results(self.polygon, self.safe_polygon, cycles_to_draw, is_static=use_static, road_geom=road)
        else:
             # Mode Editor: Redraw Editor Points
             self.map_widget.draw_editor_state(self.points)

    def toggle_visualization_mode(self, state):
        """Switches between Mobile (Optimizer) and Static (Baseline) visualization"""
        if not self.last_mission_cycles or not self.static_cycles:
            return
            
        use_static = bool(state)
        cycles_to_draw = self.static_cycles if use_static else self.last_mission_cycles
        
        # Redraw
        if self.polygon and self.safe_polygon:
            self.map_widget.draw_results(self.polygon, self.safe_polygon, cycles_to_draw, is_static=use_static)

    def show_control_panel(self):
        self.sidebar_stack.setCurrentIndex(0)

    def on_algo_toggled(self, state):
        # Optional: Warn if decomposition is experimental or show info
        pass