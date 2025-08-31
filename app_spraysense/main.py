from PySide6.QtWidgets import QApplication, QWidget, QMainWindow, QFileDialog
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from app_spraysense.GUI.SpraySenseMainWin import SpraySenseMainWindow
from shared_data import SharedData  


if __name__ == "__main__":

    # print("current cwd:")
    # print(os.getcwd())
    shared_data = SharedData.get_instance()
    app = QApplication(sys.argv)

    # Setup root logger
    logger = shared_data.logger
    logger.info("Application starting")
    
    try:
        window = SpraySenseMainWindow()
        window.show()
        sys.exit(app.exec())
    except Exception as e:
        logger.error(f"Crash report: {str(e)}", exc_info=True)

