# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'cansat-ground-stationfWvZNn.ui'
##
## Created by: Qt User Interface Compiler version 6.7.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

# Testing

from PyQt5.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PyQt5.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PyQt5.QtWidgets import (QApplication, QMainWindow, QMenuBar, QProgressBar,
    QPushButton, QSizePolicy, QStatusBar, QTabWidget,
    QWidget)

class Ui_cansatgroundstation(object):
    def setupUi(self, cansatgroundstation):
        if not cansatgroundstation.objectName():
            cansatgroundstation.setObjectName(u"cansatgroundstation")
        cansatgroundstation.setWindowModality(Qt.WindowModality.WindowModal)
        cansatgroundstation.resize(800, 600)
        self.centralwidget = QWidget(cansatgroundstation)
        self.centralwidget.setObjectName(u"centralwidget")
        self.Activate = QPushButton(self.centralwidget)
        self.Activate.setObjectName(u"Activate")
        self.Activate.setGeometry(QRect(20, 110, 100, 32))
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setGeometry(QRect(20, 10, 120, 80))
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        self.progressBar = QProgressBar(self.centralwidget)
        self.progressBar.setObjectName(u"progressBar")
        self.progressBar.setGeometry(QRect(590, 20, 161, 23))
        self.progressBar.setValue(24)
        cansatgroundstation.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(cansatgroundstation)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 800, 24))
        cansatgroundstation.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(cansatgroundstation)
        self.statusbar.setObjectName(u"statusbar")
        cansatgroundstation.setStatusBar(self.statusbar)

        self.retranslateUi(cansatgroundstation)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(cansatgroundstation)
    # setupUi

    def retranslateUi(self, cansatgroundstation):
        cansatgroundstation.setWindowTitle(QCoreApplication.translate("cansatgroundstation", u"MainWindow", None))
        self.Activate.setText(QCoreApplication.translate("cansatgroundstation", u"Activate", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("cansatgroundstation", u"Tab 1", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("cansatgroundstation", u"Tab 2", None))
    # retranslateUi

