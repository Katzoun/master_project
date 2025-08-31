from PySide6.QtWidgets import QApplication, QWidget, QMainWindow, QFileDialog
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from app_scan.GUI.sidebar import MainWindowWithSideBar


if __name__ == "__main__":
    #print current cwd
    print("current cwd:")
    print(os.getcwd())

    app = QApplication(sys.argv)
    window = MainWindowWithSideBar()
    window.show()
    app.exec()