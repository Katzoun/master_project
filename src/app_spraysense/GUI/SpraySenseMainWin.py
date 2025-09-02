from PySide6.QtWidgets import QWidget, QMainWindow, QFileDialog
import os
import sys
from app_spraysense.GUI.ui_mainwin import Ui_MainWindow

class SpraySenseMainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        # self.setWindowTitle("My Application")

        self.sidebar_expanded.setHidden(True)

        self.pb_dashboard_small.clicked.connect(lambda: self.switch_to_page(self.page_dashboard))
        self.pb_setup_small.clicked.connect(lambda: self.switch_to_page(self.page_setup))
        self.pb_edit_small.clicked.connect(lambda: self.switch_to_page(self.page_edit))
        self.pb_capture_small.clicked.connect(lambda: self.switch_to_page(self.page_capture))
        self.pb_unknown1_small.clicked.connect(lambda: self.switch_to_page(self.page_none))
        self.pb_settings_small.clicked.connect(lambda: self.switch_to_page(self.page_settings))


        self.pb_dashboard_expanded.clicked.connect(lambda: self.switch_to_page(self.page_dashboard))
        self.pb_setup_expanded.clicked.connect(lambda: self.switch_to_page(self.page_setup))
        self.pb_edit_expanded.clicked.connect(lambda: self.switch_to_page(self.page_edit))
        self.pb_capture_expanded.clicked.connect(lambda: self.switch_to_page(self.page_capture))
        self.pb_unknown1_expanded.clicked.connect(lambda: self.switch_to_page(self.page_none))
        self.pb_settings_expanded.clicked.connect(lambda: self.switch_to_page(self.page_settings))




    def switch_to_page(self, page_widget: QWidget):
        print("Switching to page:", page_widget.objectName())
        self.stackedWidget.setCurrentWidget(page_widget)



