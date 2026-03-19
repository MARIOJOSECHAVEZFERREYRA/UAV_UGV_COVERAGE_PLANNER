DARK_BLUE = "#2C3E50"
MID_BLUE = "#34495e"
TEXT_WHITE = "#ecf0f1"
TEXT_GRAY = "#bdc3c7"
ACCENT_GREEN = "#2ecc71"
ACCENT_ORANGE = "#f39c12"
ACCENT_RED = "#e74c3c"
ACCENT_BLUE = "#3498db"

# Map Colors
MAP_FIELD_BORDER = "#2ecc71"
MAP_FIELD_FILL = "#abebc6" # Lighter green for fill

MAP_MARKER_START = "#2ecc71" # Green
MAP_MARKER_END = "#e74c3c"   # Red
MAP_MARKER_TRUCK = "#f39c12" # Orange

MAP_ROUTE_TRUCK = "#e67e22"  # Orange dashed line

# High contrast cycle colors
MAP_CYCLE_COLORS = [
    '#2980b9', # Blue
    '#8e44ad', # Purple
    '#16a085', # Teal
    '#d35400', # Orange
    '#2c3e50', # Dark Blue
    '#c0392b'  # Red
]

SIDEBAR_STYLE = f"""
    QWidget {{
        background-color: {DARK_BLUE};
        color: {TEXT_WHITE};
        font-family: 'Segoe UI', sans-serif;
    }}
    QLabel {{
        font-size: 14px;
        font-weight: bold;
        color: {TEXT_GRAY};
        margin-top: 5px;
        margin-bottom: 5px;
    }}
    QComboBox {{
        background-color: {MID_BLUE};
        color: white;
        border: 1px solid #7f8c8d;
        border-radius: 4px;
        padding: 6px;
        font-size: 13px;
    }}
    QComboBox::drop-down {{ border: 0px; }}
    QTextEdit {{
        background-color: {MID_BLUE};
        color: {ACCENT_GREEN};
        border: 1px solid #7f8c8d;
        border-radius: 4px;
        font-family: 'Consolas', monospace;
        font-size: 12px;
    }}
"""

#button styles
BTN_CALC_STYLE = f"""
    QPushButton {{
        background-color: {ACCENT_GREEN};
        color: {DARK_BLUE};
        font-size: 15px;
        font-weight: 900;
        border-radius: 6px;
        border: 2px solid #27ae60;
    }}
    QPushButton:hover {{ background-color: #27ae60; color: white; }}
    QPushButton:pressed {{ background-color: #219150; margin-top: 2px; }}
"""

BTN_EXPORT_STYLE = f"""
    QPushButton {{
        background-color: {ACCENT_ORANGE};
        color: white; font-weight: bold; font-size: 14px; border-radius: 4px;
    }}
    QPushButton:hover {{ background-color: #d35400; }}
    QPushButton:disabled {{ background-color: #7f8c8d; color: #bdc3c7; }}
"""

BTN_CLEAR_STYLE = f"""
    QPushButton {{
        background-color: {ACCENT_RED}; 
        color: white; border-radius: 4px; padding: 10px; font-weight: bold;
    }}
    QPushButton:hover {{ background-color: #c0392b; }}
"""

BTN_LOAD_STYLE = f"""
    QPushButton {{
        background-color: {ACCENT_BLUE}; 
        color: white; border-radius: 4px; padding: 10px; font-weight: bold;
    }}
    QPushButton:hover {{ background-color: #2980b9; }}
"""

QMESSAGEBOX_STYLE = f"""
    QMessageBox {{
        background-color: {DARK_BLUE};
        color: {TEXT_WHITE};
    }}
    QMessageBox QLabel {{
        color: {TEXT_WHITE};
        font-size: 13px;
    }}
    QMessageBox QPushButton {{
        background-color: {MID_BLUE};
        color: white;
        border: 1px solid #7f8c8d;
        border-radius: 4px;
        padding: 5px 15px;
    }}
    QMessageBox QPushButton:hover {{
        background-color: {ACCENT_BLUE};
        color: white;
    }}
"""