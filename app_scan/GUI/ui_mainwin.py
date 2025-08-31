# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwin.ui'
##
## Created by: Qt User Interface Compiler version 6.9.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QSizePolicy, QSpacerItem,
    QStackedWidget, QVBoxLayout, QWidget)
import app_scan.GUI.icons_rc

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1600, 1000)
        MainWindow.setMinimumSize(QSize(1600, 800))
        MainWindow.setStyleSheet(u"background-color: rgb(200, 200, 200);\n"
"")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.centralwidget.setStyleSheet(u"")
        self.gridLayout = QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.sidebar_small = QWidget(self.centralwidget)
        self.sidebar_small.setObjectName(u"sidebar_small")
        self.sidebar_small.setStyleSheet(u"QWidget{\n"
"	background-color: rgb(30, 30, 30);\n"
"}\n"
"QPushButton{\n"
"background-color: rgb(200, 200, 200);\n"
"color:white;\n"
"text-align:center;\n"
"height:30px;\n"
"border: 3px solid rgb(200, 200, 200);   \n"
"border-radius:15px;\n"
"padding-left:3px;\n"
"padding-right:3px;\n"
"\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: rgb(100, 100, 100);  /* p\u016fvodn\u00ed barva <span> */\n"
"    border: 3px solid rgb(70,70, 70);\n"
"      /* aby se po p\u0159id\u00e1n\u00ed borderu nezv\u011bt\u0161ilo tla\u010d\u00edtko */\n"
"}\n"
"\n"
"/* Aktivn\u00ed stav (klik) \u2013 m\u00edrn\u011b zesv\u011btl\u00edme okraj */\n"
"QPushButton:pressed {\n"
"    background-color:  rgb(120, 120, 120); \n"
"    border: 3px solid  rgb(120, 120, 120); \n"
"\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color:  rgb(120, 120, 120); \n"
"    border: 3px solid  rgb(120, 120, 120); \n"
"\n"
"}")
        self.verticalLayout_3 = QVBoxLayout(self.sidebar_small)
        self.verticalLayout_3.setSpacing(15)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(9, 9, 9, 9)
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setSpacing(10)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.pb_menu_small = QPushButton(self.sidebar_small)
        self.pb_menu_small.setObjectName(u"pb_menu_small")
        self.pb_menu_small.setStyleSheet(u"QPushButton{\n"
"background-color: rgb(200, 200, 200);\n"
"color:white;\n"
"text-align:center;\n"
"height:30px;\n"
"border: 3px solid rgb(200, 200, 200);   \n"
"border-radius:15px;\n"
"\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: rgb(100, 100, 100);  /* p\u016fvodn\u00ed barva <span> */\n"
"    border: 3px solid rgb(70,70, 70);\n"
"      /* aby se po p\u0159id\u00e1n\u00ed borderu nezv\u011bt\u0161ilo tla\u010d\u00edtko */\n"
"}\n"
"\n"
"/* Aktivn\u00ed stav (klik) \u2013 m\u00edrn\u011b zesv\u011btl\u00edme okraj */\n"
"QPushButton:pressed {\n"
"    background-color:  rgb(70, 70, 70); \n"
"    border: 3px solid  rgb(70, 70, 70); \n"
"\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color:  rgb(200, 200, 200); \n"
"    border: 3px solid  rgb(200, 200, 200); \n"
"\n"
"}")
        icon = QIcon()
        icon.addFile(u":/Icons/menu.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.pb_menu_small.setIcon(icon)
        self.pb_menu_small.setIconSize(QSize(25, 25))
        self.pb_menu_small.setCheckable(True)

        self.verticalLayout.addWidget(self.pb_menu_small)

        self.pb_dashboard_small = QPushButton(self.sidebar_small)
        self.pb_dashboard_small.setObjectName(u"pb_dashboard_small")
        icon1 = QIcon()
        icon1.addFile(u":/Icons/dashboard-monitor.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.pb_dashboard_small.setIcon(icon1)
        self.pb_dashboard_small.setIconSize(QSize(25, 25))
        self.pb_dashboard_small.setCheckable(True)
        self.pb_dashboard_small.setAutoExclusive(True)

        self.verticalLayout.addWidget(self.pb_dashboard_small)

        self.pb_setup_small = QPushButton(self.sidebar_small)
        self.pb_setup_small.setObjectName(u"pb_setup_small")
        icon2 = QIcon()
        icon2.addFile(u":/Icons/circle-nodes.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.pb_setup_small.setIcon(icon2)
        self.pb_setup_small.setIconSize(QSize(25, 25))
        self.pb_setup_small.setCheckable(True)
        self.pb_setup_small.setAutoExclusive(True)

        self.verticalLayout.addWidget(self.pb_setup_small)

        self.pb_edit_small = QPushButton(self.sidebar_small)
        self.pb_edit_small.setObjectName(u"pb_edit_small")
        icon3 = QIcon()
        icon3.addFile(u":/Icons/edit (1).png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.pb_edit_small.setIcon(icon3)
        self.pb_edit_small.setIconSize(QSize(25, 25))
        self.pb_edit_small.setCheckable(True)
        self.pb_edit_small.setAutoExclusive(True)

        self.verticalLayout.addWidget(self.pb_edit_small)

        self.pb_capture_small = QPushButton(self.sidebar_small)
        self.pb_capture_small.setObjectName(u"pb_capture_small")
        icon4 = QIcon()
        icon4.addFile(u":/Icons/camera.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.pb_capture_small.setIcon(icon4)
        self.pb_capture_small.setIconSize(QSize(25, 25))
        self.pb_capture_small.setCheckable(True)
        self.pb_capture_small.setAutoExclusive(True)

        self.verticalLayout.addWidget(self.pb_capture_small)

        self.pb_unknown1_small = QPushButton(self.sidebar_small)
        self.pb_unknown1_small.setObjectName(u"pb_unknown1_small")
        self.pb_unknown1_small.setIconSize(QSize(25, 25))
        self.pb_unknown1_small.setCheckable(True)
        self.pb_unknown1_small.setAutoExclusive(True)

        self.verticalLayout.addWidget(self.pb_unknown1_small)


        self.verticalLayout_3.addLayout(self.verticalLayout)

        self.sidebar_spacer_expanded_2 = QSpacerItem(20, 465, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_3.addItem(self.sidebar_spacer_expanded_2)

        self.pb_settings_small = QPushButton(self.sidebar_small)
        self.pb_settings_small.setObjectName(u"pb_settings_small")
        icon5 = QIcon()
        icon5.addFile(u":/Icons/settings.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.pb_settings_small.setIcon(icon5)
        self.pb_settings_small.setIconSize(QSize(25, 25))
        self.pb_settings_small.setCheckable(True)
        self.pb_settings_small.setAutoExclusive(True)

        self.verticalLayout_3.addWidget(self.pb_settings_small)

        self.label_8 = QLabel(self.sidebar_small)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setPixmap(QPixmap(u":/Icons/vutlogo.png"))

        self.verticalLayout_3.addWidget(self.label_8)


        self.gridLayout.addWidget(self.sidebar_small, 0, 0, 1, 1)

        self.sidebar_expanded = QWidget(self.centralwidget)
        self.sidebar_expanded.setObjectName(u"sidebar_expanded")
        self.sidebar_expanded.setMinimumSize(QSize(150, 0))
        self.sidebar_expanded.setStyleSheet(u"QWidget{\n"
"	background-color: rgb(30, 30, 30);\n"
"}\n"
"QPushButton{\n"
"background-color: rgb(200, 200, 200);\n"
"color:black;\n"
"text-align:left;\n"
"height:30px;\n"
"border: 3px solid rgb(200, 200, 200);   \n"
"padding-left:10px;\n"
"border-top-left-radius:15px;\n"
"border-bottom-left-radius:15px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: rgb(100, 100, 100);  /* p\u016fvodn\u00ed barva <span> */\n"
"    border: 3px solid rgb(70,70, 70);\n"
"      /* aby se po p\u0159id\u00e1n\u00ed borderu nezv\u011bt\u0161ilo tla\u010d\u00edtko */\n"
"}\n"
"\n"
"/* Aktivn\u00ed stav (klik) \u2013 m\u00edrn\u011b zesv\u011btl\u00edme okraj */\n"
"QPushButton:pressed {\n"
"    background-color:  rgb(120, 120, 120); \n"
"    border: 3px solid  rgb(120, 120, 120); \n"
"\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color:  rgb(120, 120, 120); \n"
"    border: 3px solid  rgb(120, 120, 120); \n"
"\n"
"}\n"
"")
        self.verticalLayout_4 = QVBoxLayout(self.sidebar_expanded)
        self.verticalLayout_4.setSpacing(15)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(-1, -1, 0, -1)
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setSpacing(10)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.pb_menu_expanded = QPushButton(self.sidebar_expanded)
        self.pb_menu_expanded.setObjectName(u"pb_menu_expanded")
        self.pb_menu_expanded.setStyleSheet(u"QPushButton{\n"
"background-color: rgb(200, 200, 200);\n"
"color:white;\n"
"text-align:center;\n"
"height:30px;\n"
"border: 3px solid rgb(200, 200, 200);   \n"
"border-top-left-radius:15px;\n"
"border-bottom-left-radius:15px;\n"
"\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color:  rgb(200, 200, 200); \n"
"    border: 3px solid  rgb(200, 200, 200); \n"
"\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background: rgb(100, 100, 100);  /* p\u016fvodn\u00ed barva <span> */\n"
"    border: 3px solid rgb(70,70, 70);\n"
"      /* aby se po p\u0159id\u00e1n\u00ed borderu nezv\u011bt\u0161ilo tla\u010d\u00edtko */\n"
"}\n"
"\n"
"/* Aktivn\u00ed stav (klik) \u2013 m\u00edrn\u011b zesv\u011btl\u00edme okraj */\n"
"QPushButton:pressed {\n"
"    background-color:  rgb(70, 70, 70); \n"
"    border: 3px solid  rgb(70, 70, 70); \n"
"\n"
"}\n"
"")
        self.pb_menu_expanded.setIcon(icon)
        self.pb_menu_expanded.setIconSize(QSize(25, 25))
        self.pb_menu_expanded.setCheckable(True)

        self.verticalLayout_2.addWidget(self.pb_menu_expanded)

        self.pb_dashboard_expanded = QPushButton(self.sidebar_expanded)
        self.pb_dashboard_expanded.setObjectName(u"pb_dashboard_expanded")
        self.pb_dashboard_expanded.setIcon(icon1)
        self.pb_dashboard_expanded.setIconSize(QSize(25, 25))
        self.pb_dashboard_expanded.setCheckable(True)
        self.pb_dashboard_expanded.setChecked(True)
        self.pb_dashboard_expanded.setAutoExclusive(True)

        self.verticalLayout_2.addWidget(self.pb_dashboard_expanded)

        self.pb_setup_expanded = QPushButton(self.sidebar_expanded)
        self.pb_setup_expanded.setObjectName(u"pb_setup_expanded")
        self.pb_setup_expanded.setIcon(icon2)
        self.pb_setup_expanded.setIconSize(QSize(25, 25))
        self.pb_setup_expanded.setCheckable(True)
        self.pb_setup_expanded.setAutoExclusive(True)

        self.verticalLayout_2.addWidget(self.pb_setup_expanded)

        self.pb_edit_expanded = QPushButton(self.sidebar_expanded)
        self.pb_edit_expanded.setObjectName(u"pb_edit_expanded")
        self.pb_edit_expanded.setIcon(icon3)
        self.pb_edit_expanded.setIconSize(QSize(25, 25))
        self.pb_edit_expanded.setCheckable(True)
        self.pb_edit_expanded.setAutoExclusive(True)

        self.verticalLayout_2.addWidget(self.pb_edit_expanded)

        self.pb_capture_expanded = QPushButton(self.sidebar_expanded)
        self.pb_capture_expanded.setObjectName(u"pb_capture_expanded")
        self.pb_capture_expanded.setIcon(icon4)
        self.pb_capture_expanded.setIconSize(QSize(25, 25))
        self.pb_capture_expanded.setCheckable(True)
        self.pb_capture_expanded.setAutoExclusive(True)

        self.verticalLayout_2.addWidget(self.pb_capture_expanded)

        self.pb_unknown1_expanded = QPushButton(self.sidebar_expanded)
        self.pb_unknown1_expanded.setObjectName(u"pb_unknown1_expanded")
        self.pb_unknown1_expanded.setIconSize(QSize(25, 25))
        self.pb_unknown1_expanded.setCheckable(True)
        self.pb_unknown1_expanded.setAutoExclusive(True)

        self.verticalLayout_2.addWidget(self.pb_unknown1_expanded)


        self.verticalLayout_4.addLayout(self.verticalLayout_2)

        self.sidebar_spacer_expanded = QSpacerItem(20, 465, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_4.addItem(self.sidebar_spacer_expanded)

        self.pb_settings_expanded = QPushButton(self.sidebar_expanded)
        self.pb_settings_expanded.setObjectName(u"pb_settings_expanded")
        self.pb_settings_expanded.setIcon(icon5)
        self.pb_settings_expanded.setIconSize(QSize(25, 25))
        self.pb_settings_expanded.setCheckable(True)
        self.pb_settings_expanded.setAutoExclusive(True)

        self.verticalLayout_4.addWidget(self.pb_settings_expanded)

        self.label_7 = QLabel(self.sidebar_expanded)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setPixmap(QPixmap(u":/Icons/uailogofull.png"))

        self.verticalLayout_4.addWidget(self.label_7)


        self.gridLayout.addWidget(self.sidebar_expanded, 0, 1, 1, 1)

        self.main_okno = QWidget(self.centralwidget)
        self.main_okno.setObjectName(u"main_okno")
        self.verticalLayout_5 = QVBoxLayout(self.main_okno)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.top_bar = QWidget(self.main_okno)
        self.top_bar.setObjectName(u"top_bar")
        self.horizontalLayout = QHBoxLayout(self.top_bar)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.pushButton_13 = QPushButton(self.top_bar)
        self.pushButton_13.setObjectName(u"pushButton_13")
        self.pushButton_13.setCheckable(True)

        self.horizontalLayout.addWidget(self.pushButton_13)

        self.pushButton_14 = QPushButton(self.top_bar)
        self.pushButton_14.setObjectName(u"pushButton_14")

        self.horizontalLayout.addWidget(self.pushButton_14)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)


        self.verticalLayout_5.addWidget(self.top_bar)

        self.stackedWidget = QStackedWidget(self.main_okno)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.stackedWidget.setStyleSheet(u"background-color: rgb(119, 118, 123);")
        self.page_capture = QWidget()
        self.page_capture.setObjectName(u"page_capture")
        self.label_6 = QLabel(self.page_capture)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(390, 420, 67, 17))
        self.stackedWidget.addWidget(self.page_capture)
        self.page_setup = QWidget()
        self.page_setup.setObjectName(u"page_setup")
        self.label_3 = QLabel(self.page_setup)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(300, 450, 67, 17))
        self.stackedWidget.addWidget(self.page_setup)
        self.page_dashboard = QWidget()
        self.page_dashboard.setObjectName(u"page_dashboard")
        self.label_5 = QLabel(self.page_dashboard)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(270, 470, 67, 17))
        self.stackedWidget.addWidget(self.page_dashboard)
        self.page_settings = QWidget()
        self.page_settings.setObjectName(u"page_settings")
        self.label_2 = QLabel(self.page_settings)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(290, 460, 67, 17))
        self.stackedWidget.addWidget(self.page_settings)
        self.page_edit = QWidget()
        self.page_edit.setObjectName(u"page_edit")
        self.label_4 = QLabel(self.page_edit)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(410, 410, 67, 17))
        self.stackedWidget.addWidget(self.page_edit)
        self.page_none = QWidget()
        self.page_none.setObjectName(u"page_none")
        self.label = QLabel(self.page_none)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(330, 346, 141, 81))
        self.stackedWidget.addWidget(self.page_none)

        self.verticalLayout_5.addWidget(self.stackedWidget)


        self.gridLayout.addWidget(self.main_okno, 0, 2, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.pb_unknown1_expanded.toggled.connect(self.pb_unknown1_small.setChecked)
        self.pb_capture_expanded.toggled.connect(self.pb_capture_small.setChecked)
        self.pb_edit_expanded.toggled.connect(self.pb_edit_small.setChecked)
        self.pb_setup_expanded.toggled.connect(self.pb_setup_small.setChecked)
        self.pb_dashboard_expanded.toggled.connect(self.pb_dashboard_small.setChecked)
        self.pb_menu_expanded.toggled.connect(self.pb_menu_small.setChecked)
        self.pb_unknown1_small.toggled.connect(self.pb_unknown1_expanded.setChecked)
        self.pb_capture_small.toggled.connect(self.pb_capture_expanded.setChecked)
        self.pb_edit_small.toggled.connect(self.pb_edit_expanded.setChecked)
        self.pb_setup_small.toggled.connect(self.pb_setup_expanded.setChecked)
        self.pb_dashboard_small.toggled.connect(self.pb_dashboard_expanded.setChecked)
        self.pb_menu_small.toggled.connect(self.pb_menu_expanded.setChecked)
        self.pb_menu_expanded.toggled.connect(self.sidebar_small.setHidden)
        self.pb_menu_expanded.toggled.connect(self.sidebar_expanded.setVisible)
        self.pb_settings_expanded.toggled.connect(self.pb_settings_small.setChecked)
        self.pb_settings_small.toggled.connect(self.pb_settings_expanded.setChecked)

        self.stackedWidget.setCurrentIndex(3)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.pb_menu_small.setText("")
        self.pb_dashboard_small.setText("")
        self.pb_setup_small.setText("")
        self.pb_edit_small.setText("")
        self.pb_capture_small.setText("")
        self.pb_unknown1_small.setText("")
        self.pb_settings_small.setText("")
        self.label_8.setText("")
        self.pb_menu_expanded.setText("")
        self.pb_dashboard_expanded.setText(QCoreApplication.translate("MainWindow", u"Dashboard", None))
        self.pb_setup_expanded.setText(QCoreApplication.translate("MainWindow", u"Setup", None))
        self.pb_edit_expanded.setText(QCoreApplication.translate("MainWindow", u"Edit", None))
        self.pb_capture_expanded.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.pb_unknown1_expanded.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.pb_settings_expanded.setText(QCoreApplication.translate("MainWindow", u"Settings", None))
        self.label_7.setText("")
        self.pushButton_13.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.pushButton_14.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"capture", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"setup", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"dashboards", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"settings", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"edit", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"None", None))
    # retranslateUi

