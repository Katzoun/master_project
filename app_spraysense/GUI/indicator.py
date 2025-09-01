from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt

class Indicator(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(20, 20)
        self._color = QtGui.QColor('red')

    def setColor(self, color: QtGui.QColor):
        self._color = color
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setBrush(QtGui.QBrush(self._color))
        painter.setPen(Qt.NoPen)
        radius = min(self.width(), self.height()) / 2
        painter.drawEllipse(QtCore.QPointF(self.width() / 2, self.height() / 2), radius, radius)