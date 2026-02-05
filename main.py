import sys
import os
from PyQt6.QtWidgets import QApplication

# Configure path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from gui.app_window import AgriSwarmApp
# Import data to initialize DB
from data import drone_data 

def main():
    # Ensure directories exist
    base_dir = os.path.dirname(__file__)
    data_dir = os.path.join(base_dir, 'data')
    if not os.path.exists(data_dir): os.makedirs(data_dir)
    filename = os.path.join(data_dir, 'mi_campo_dibujado.json')

    # Initialize Qt App
    app = QApplication(sys.argv)
    app.setStyle("Fusion") # Force Fusion style for consistent element rendering
    
    # Create main window
    window = AgriSwarmApp(filename=filename)
    window.show() # Show window

    # Start event loop
    sys.exit(app.exec())

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        with open("crash.log", "w") as f:
            f.write(traceback.format_exc())
            f.write(str(e))
        print(f"CRASH: {e}")