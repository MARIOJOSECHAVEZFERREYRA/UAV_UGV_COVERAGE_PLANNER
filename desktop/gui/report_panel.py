from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget, QTableWidgetItem, 
    QHeaderView, QFrame, QPushButton, QScrollArea
)
from PyQt6.QtGui import QColor, QFont
from PyQt6.QtCore import Qt, pyqtSignal
from gui.styles import DARK_BLUE, ACCENT_GREEN, ACCENT_ORANGE, TEXT_WHITE, ACCENT_BLUE

class ReportPanel(QWidget):
    back_clicked = pyqtSignal()

    def __init__(self, comprehensive_data, comparison_data, resource_data, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {DARK_BLUE};")
        
        # Scroll Area because reports can be long
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0,0,0,0)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(f"background-color: {DARK_BLUE}; border: none;")
        
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)
        layout.setSpacing(15)
        layout.setContentsMargins(15, 20, 15, 20)
        
        # --- HEADER ---
        lbl_title = QLabel("TECHNICAL REPORT")
        lbl_title.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {ACCENT_ORANGE};")
        lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_title)
        
        
        # --- RESUMEN EJECUTIVO (KPIs) ---
        self.add_separator(layout)
        layout.addWidget(QLabel("EXECUTIVE SUMMARY:"))
        
        row1 = QHBoxLayout()
        # Prod
        prod = comprehensive_data.get('productivity_ha_hr', 0)
        self.create_mini_element(row1, "PRODUCTIVITY", f"{prod:.1f} ha/h", ACCENT_GREEN)
        # Eficiencia
        eff = comprehensive_data.get('efficiency_ratio', 0)
        self.create_mini_element(row1, "EFFICIENCY", f"{eff:.0f}%", ACCENT_ORANGE)
        layout.addLayout(row1)
        
        row2 = QHBoxLayout()
        # Tiempo
        time = comprehensive_data.get('total_op_time_min', 0)
        self.create_mini_element(row2, "TOTAL TIME", f"{time:.0f} min", "white")
        # Dosis
        dose = comprehensive_data.get('real_dosage_l_ha', 0)
        self.create_mini_element(row2, "REAL DOSAGE", f"{dose:.1f} L/ha", ACCENT_BLUE)
        layout.addLayout(row2)

        row3 = QHBoxLayout()
        # Distancia Total
        s_km = comprehensive_data.get('spray_dist_km', 0)
        d_km = comprehensive_data.get('dead_dist_km', 0)
        total_km = s_km + d_km
        self.create_mini_element(row3, "ROUTE DISTANCE", f"{total_km:.2f} km", "white")
        
        # Consumo Estimado (Mezcla)
        mix = resource_data.get('total_mix_l', 0)
        self.create_mini_element(row3, "TOTAL MIX", f"{mix:.1f} L", ACCENT_BLUE)
        layout.addLayout(row3)
        
        row4 = QHBoxLayout()
        # Baterias
        packs = resource_data.get('battery_packs', 0)
        self.create_mini_element(row4, "BATTERIES REQ.", f"{packs} Packs", ACCENT_ORANGE)
        
        # Ciclos (Recargas) - Inferido de stops
        stops_count = len(resource_data.get('stops', []))
        recargas = stops_count - 1 if stops_count > 0 else 0
        self.create_mini_element(row4, "REFILLS", f"{recargas}", "white")
        layout.addLayout(row4)

        # --- TABLA COMPARATIVA ---
        self.add_separator(layout)
        lbl_comp = QLabel("DETAILED COMPARISON")
        lbl_comp.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {ACCENT_ORANGE}; margin-top: 5px;")
        lbl_comp.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_comp)

        table = QTableWidget(3, 4)
        table.setHorizontalHeaderLabels(["Metric", "Static", "Mobile", "Impact"])
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        # Resize first column to content if needed, but Stretch is fine for now
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        table.setStyleSheet(f"""
            QTableWidget {{ background-color: transparent; border: 1px solid #444; gridline-color: #555; color: white; font-size: 11px; }}
            QHeaderView::section {{ background-color: #2c3e50; color: white; border: 1px solid #555; padding: 2px; font-weight: bold; }}
            QTableWidget::item {{ padding: 2px; }}
        """)
        
        # Helper to set item
        def set_item(r, c, text, color="white", bold=False):
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            item.setForeground(QColor(color))
            if bold: item.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
            table.setItem(r, c, item)

        # 1. Distancia Muerta (Dron) NO PRODUCTIVA
        s_dead = comparison_data.get('static_dead_km', 0)
        m_dead = comparison_data.get('mobile_dead_km', 0)
        diff_dead = comparison_data.get('savings_km', 0)
        set_item(0, 0, "Dead Dist. (Drone)", "#bdc3c7")
        set_item(0, 1, f"{s_dead:.2f} km")
        set_item(0, 2, f"{m_dead:.2f} km")
        set_item(0, 3, f"-{diff_dead:.2f} km", ACCENT_GREEN, True)

        # 2. Eficiencia Global
        s_eff = comparison_data.get('efficiency_static_pct', 0)
        m_eff = comparison_data.get('efficiency_mobile_pct', 0)
        diff_eff = comparison_data.get('efficiency_gain_pct', 0)
        set_item(1, 0, "Flight Efficiency", "#bdc3c7")
        set_item(1, 1, f"{s_eff:.1f}%")
        set_item(1, 2, f"{m_eff:.1f}%")
        set_item(1, 3, f"+{diff_eff:.1f}%", ACCENT_GREEN, True)

        # 3. Recorrido Estacion (Truck)
        s_truck = comparison_data.get('static_truck_km', 0)
        m_truck = comparison_data.get('mobile_truck_km', 0)
        diff_truck = m_truck - s_truck
        set_item(2, 0, "Station Travel", "#bdc3c7")
        set_item(2, 1, f"{s_truck:.2f} km")
        set_item(2, 2, f"{m_truck:.2f} km")
        set_item(2, 3, f"+{diff_truck:.2f} km", ACCENT_ORANGE, True) # Extra cost

        table.setFixedHeight(120) 
        layout.addWidget(table)

        # --- LOGISTICA ---
        self.add_separator(layout)
        
        lbl_log = QLabel("RESOURCE PLAN")
        lbl_log.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {TEXT_WHITE}; margin-top: 10px;")
        layout.addWidget(lbl_log)
        
        # (Removed duplicated info here)
        
        # --- TABLA ---
        self.create_stops_table(layout, resource_data.get('stops', []))
        
        # --- ESPACIO EXTRA ---
        layout.addStretch()
        
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)
        
        # --- FOOTER (BOTON VOLVER) ---
        footer_container = QWidget()
        fl = QVBoxLayout(footer_container)
        btn_back = QPushButton("BACK TO PANEL")
        btn_back.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_back.setStyleSheet(f"""
            QPushButton {{
                background-color: {ACCENT_GREEN}; color: {DARK_BLUE}; 
                padding: 12px; border-radius: 4px; font-weight: bold; font-size: 14px;
            }}
            QPushButton:hover {{ background-color: #27ae60; color: white; }}
        """)
        btn_back.clicked.connect(self.back_clicked.emit)
        fl.addWidget(btn_back)
        
        main_layout.addWidget(footer_container)

    def create_mini_element(self, layout, title, value, color):
        v = QVBoxLayout()
        v.setSpacing(2)
        l1 = QLabel(title)
        l1.setStyleSheet("color: #bdc3c7; font-size: 9px; font-weight: bold;")
        l1.setAlignment(Qt.AlignmentFlag.AlignCenter)
        l2 = QLabel(value)
        l2.setStyleSheet(f"color: {color}; font-size: 16px; font-weight: bold;")
        l2.setAlignment(Qt.AlignmentFlag.AlignCenter)
        v.addWidget(l1)
        v.addWidget(l2)
        layout.addLayout(v)

    def add_separator(self, layout):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Plain)
        line.setStyleSheet("background-color: #7f8c8d;")
        layout.addWidget(line)

    def create_stops_table(self, parent_layout, stops):
        # Simplified for sidebar
        table = QTableWidget()
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(["STOP", "ACTION"])
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        table.verticalHeader().setVisible(False)
        table.setStyleSheet(f"""
            QTableWidget {{
                background-color: #34495e; color: white;
                gridline-color: #7f8c8d; border: none;
            }}
            QHeaderView::section {{
                background-color: {DARK_BLUE}; color: #bdc3c7; border: none; font-size: 11px;
            }}
        """)
        
        table.setRowCount(len(stops))
        for i, stop in enumerate(stops):
            table.setItem(i, 0, QTableWidgetItem(stop['name']))
            table.setItem(i, 1, QTableWidgetItem(stop['action']))
            
        parent_layout.addWidget(table)
