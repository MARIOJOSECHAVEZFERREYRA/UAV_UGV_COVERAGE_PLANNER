"""
UI Builder Module for AgriSwarm Planner
Separates UI construction logic from the main application window.
"""

from PyQt6.QtWidgets import (QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QPushButton, 
                              QFrame, QFormLayout, QDoubleSpinBox, QCheckBox)
from PyQt6.QtCore import Qt
from gui.styles import *
from data import DroneDB


class UIBuilder:
    """Helper class to construct UI components for the main application window"""
    
    @staticmethod
    def create_sidebar_header(layout):
        """Create the branding header for the sidebar"""
        lbl_brand = QLabel("AGRISWARM")
        lbl_brand.setStyleSheet("font-size: 26px; color: #ecf0f1; letter-spacing: 2px; margin-bottom: 10px;")
        lbl_brand.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_brand)
    
    @staticmethod
    def create_drone_selector(layout, on_change_callback):
        """Create drone selection combo box"""
        combo_drones = QComboBox()
        combo_drones.addItems(DroneDB.get_drone_names())
        combo_drones.setCurrentText("DJI Agras T30")
        combo_drones.currentTextChanged.connect(on_change_callback)
        layout.addWidget(combo_drones)
        return combo_drones
    
    @staticmethod
    def create_mission_parameters(layout):
        """Create mission parameter controls (swath, tank, speed, etc.)"""
        from PyQt6.QtWidgets import QFormLayout, QLabel, QSizePolicy
        
        # Use standard FormLayout - it handles platform alignment best
        params_layout = QFormLayout()
        params_layout.setSpacing(10)
        params_layout.setContentsMargins(0, 5, 0, 5)
        
        from PyQt6.QtGui import QPalette, QColor

        def create_spinbox():
            sb = QDoubleSpinBox()
            
            # Use Palette for colors ensures standard arrows are drawn by Fusion style
            palette = sb.palette()
            palette.setColor(QPalette.ColorRole.Base, QColor("#34495e"))
            palette.setColor(QPalette.ColorRole.Text, QColor("white"))
            palette.setColor(QPalette.ColorRole.Button, QColor("#34495e")) # For the button background
            sb.setPalette(palette)
            
            
            # CRITICAL: Prevent vertical stretching/squashing
            sb.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            sb.setMinimumHeight(30)  
            return sb

        def add_row(text, widget):
            lbl = QLabel(text)
            # Ensure label is visible
            lbl.setStyleSheet("color: #bdc3c7; font-weight: bold; font-size: 13px;")
            params_layout.addRow(lbl, widget)

        # 1. Work Width (Swath)
        spin_swath = create_spinbox()
        spin_swath.setRange(1.0, 20.0)
        spin_swath.setSingleStep(0.5)
        spin_swath.setSuffix(" m")
        add_row("Swath Width:", spin_swath)
        
        # 2. Tank
        spin_tank = create_spinbox()
        spin_tank.setRange(5.0, 100.0)
        spin_tank.setSingleStep(1.0)
        spin_tank.setSuffix(" L")
        add_row("Tank Capacity:", spin_tank)
        
        # 3. Speed
        spin_speed = create_spinbox()
        spin_speed.setRange(1.0, 15.0)
        spin_speed.setSingleStep(0.5)
        spin_speed.setSuffix(" m/s")
        add_row("Flight Speed:", spin_speed)
        
        # 4. Application Rate
        spin_app_rate = create_spinbox()
        spin_app_rate.setRange(5.0, 50.0)
        spin_app_rate.setSingleStep(1.0)
        spin_app_rate.setValue(20.0)  # Default 20 L/ha
        spin_app_rate.setSuffix(" L/ha")
        add_row("Application Rate:", spin_app_rate)
        
        # 5. Station Distance (Offset)
        spin_truck_offset = create_spinbox()
        spin_truck_offset.setRange(0.0, 50.0)
        spin_truck_offset.setSingleStep(1.0)
        spin_truck_offset.setSuffix(" m")
        spin_truck_offset.setValue(0.0)
        spin_truck_offset.setToolTip("Extra distance between field edge and truck route")
        add_row("Station Offset:", spin_truck_offset)
        
        layout.addLayout(params_layout)
        
        return {
            'swath': spin_swath,
            'tank': spin_tank,
            'speed': spin_speed,
            'app_rate': spin_app_rate,
            'truck_offset': spin_truck_offset
        }
    
    @staticmethod
    def create_geometry_buttons(layout, on_clear, on_load):
        """Create CLEAR and LOAD buttons"""
        btn_grid = QHBoxLayout()
        
        btn_clear = QPushButton("CLEAR")
        btn_clear.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_clear.setStyleSheet(BTN_CLEAR_STYLE)
        btn_clear.clicked.connect(on_clear)
        
        btn_load = QPushButton("LOAD")
        btn_load.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_load.setStyleSheet(BTN_LOAD_STYLE)
        btn_load.clicked.connect(on_load)
        
        btn_grid.addWidget(btn_clear)
        btn_grid.addWidget(btn_load)
        layout.addLayout(btn_grid)
        
        return btn_clear, btn_load
    
    @staticmethod
    def create_custom_route_button(layout, on_toggle):
        """Create the custom route drawing button"""
        btn_draw_route = QPushButton("DRAW MOBILE ROUTE")
        btn_draw_route.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_draw_route.setStyleSheet(
            "background-color: #7f8c8d; color: white; border: none; "
            "padding: 10px; font-weight: bold; border-radius: 4px;"
        )
        btn_draw_route.setCheckable(True)
        btn_draw_route.clicked.connect(on_toggle)
        layout.addWidget(btn_draw_route)
        return btn_draw_route

    @staticmethod
    def create_obstacle_button(layout, on_toggle):
        """Create the obstacle drawing button"""
        btn_obstacle = QPushButton("ADD OBSTACLE")
        btn_obstacle.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_obstacle.setStyleSheet(
            "background-color: #7f8c8d; color: white; border: none; "
            "padding: 10px; font-weight: bold; border-radius: 4px;"
        )
        btn_obstacle.setCheckable(True)
        btn_obstacle.clicked.connect(on_toggle)
        layout.addWidget(btn_obstacle)
        return btn_obstacle
    
    @staticmethod
    def create_visual_options(layout, on_swath_toggled, on_static_toggled):
        """Create visual option checkboxes"""
        # Swath coverage checkbox
        chk_swath = QCheckBox("Show Spray Coverage (Swath)")
        chk_swath.setChecked(False)  # Disabled by default for performance
        chk_swath.stateChanged.connect(on_swath_toggled)
        layout.addWidget(chk_swath)
        
        # Static vs Mobile visualization toggle
        chk_mode_static = QCheckBox("Visualize: Static Station")
        chk_mode_static.setChecked(False)
        chk_mode_static.setEnabled(False)  # Only active after calculation
        chk_mode_static.setStyleSheet("color: #e74c3c; font-weight: bold;")
        chk_mode_static.stateChanged.connect(on_static_toggled)
        layout.addWidget(chk_mode_static)
        
        return chk_swath, chk_mode_static
    
    @staticmethod
    def create_algorithm_selector(layout, on_algo_toggled):
        """Create algorithm selection checkbox"""
        chk_algo = QCheckBox("Use Formal Decomposition (Thesis)")
        chk_algo.setStyleSheet("color: #2ecc71; font-weight: bold;")
        chk_algo.setToolTip("Enables Boustrophedon Cellular Decomposition for complex obstacles")
        chk_algo.stateChanged.connect(on_algo_toggled)
        layout.addWidget(chk_algo)
        return chk_algo
    
    @staticmethod
    def create_action_buttons(layout, on_calculate, on_report, on_export):
        """Create main action buttons (CALCULATE, REPORT, EXPORT)"""
        
        # Calculate button
        btn_calc = QPushButton("CALCULATE ROUTE")
        btn_calc.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_calc.setFixedHeight(60)
        btn_calc.setStyleSheet(BTN_CALC_STYLE)
        btn_calc.clicked.connect(on_calculate)
        layout.addWidget(btn_calc)
        
        # Comparative report button
        btn_report = QPushButton("VIEW COMPARATIVE REPORT")
        btn_report.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_report.setFixedHeight(40)
        btn_report.setStyleSheet(
            f"background-color: {DARK_BLUE}; color: {ACCENT_ORANGE}; font-weight: bold; "
            f"border: 1px solid {ACCENT_ORANGE}; border-radius: 4px;"
        )
        btn_report.clicked.connect(on_report)
        btn_report.setEnabled(False)  # Enable only after calculation
        layout.addWidget(btn_report)
        
        # Export button
        btn_export = QPushButton("EXPORT .PLAN")
        btn_export.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_export.setFixedHeight(45)
        btn_export.setStyleSheet(BTN_EXPORT_STYLE)
        btn_export.clicked.connect(on_export)
        btn_export.setEnabled(False)
        layout.addWidget(btn_export)
        
        return btn_calc, btn_report, btn_export
    

