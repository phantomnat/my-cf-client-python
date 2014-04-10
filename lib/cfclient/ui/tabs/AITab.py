#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

"""
This tab plots different logging data defined by configurations that has been
pre-configured.
"""

__author__ = 'Bitcraze AB'
__all__ = ['AITab']

import glob
import json
import logging
import os
import sys

logger = logging.getLogger(__name__)

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import pyqtSlot, pyqtSignal, QThread, Qt
from PyQt4.QtGui import QMessageBox
from PyQt4.QtGui import QApplication, QStyledItemDelegate, QAbstractItemView
from PyQt4.QtCore import QAbstractItemModel, QModelIndex, QString, QVariant

from pprint import pprint
import datetime

from cfclient.ui.widgets.plotwidget import PlotWidget

from cflib.crazyflie.log import Log


from cfclient.utils.controller import Controller

from cfclient.ui.tab import Tab

import threading
import socket
import SocketServer
import time
# import numpy
# import pylab

plot_tab_class = uic.loadUiType(sys.path[0] +
                                "/cfclient/ui/tabs/AITab.ui")[0]

class AITab(Tab, plot_tab_class):
    """Tab for plotting logging data"""

    _log_data_signal = pyqtSignal(int, object)

    _log_error_signal = pyqtSignal(object, str)
    _disconnected_signal = pyqtSignal(str)

    colors = ['g', 'b', 'm', 'r', 'y', 'c']

    def __init__(self, tabWidget, helper, *args):
        super(AITab, self).__init__(*args)
        self.setupUi(self)

        self.tabName = "AI"
        self.menuName = "AI"

        self._plot = PlotWidget(fps=30)
        self._plot2 = PlotWidget(fps=30)

        # Check if we could find the PyQtImport. If not, then
        # set this tab as disabled
        self.enabled = self._plot.can_enable

        # self.dataSelector.setModel(self._model)
        # self._log_data_signal.connect(self._log_data_received)
        self.tabWidget = tabWidget
        self.helper = helper

        self.plotLayout.addWidget(self._plot)
        # self.plotLayout2.addWidget(self._plot2)

        self._previous_config = None
        self._started_previous = False

        # initial TCP Server thread
        # from cfclient.utils.tcpserver import TcpServerThread

        self.controller = Controller(helper.cf)

        # self.controller.start(QThread.HighestPriority)
        self.controller.start()

        self.last_time = time.clock()
        self._plot_time_count = 0

        # self.TcpServer.ClientConnected.add_callback(self._client_connected)


        # self.controller = Controller(self, helper.cf)
        helper.inputDeviceReader.input_updated.add_callback(self.controller.update_thrust)
        # # self.controller.PlotUpdated.connect(self._data_received)
        #
        #
        # self.tcp_server = TcpServerThread()
        # self.tcp_server.data_received.add_callback(self.controller.update_input)
        # self.tcp_server.data_received.add_callback(self._data_received)
        #
        # self.tcp_server.start()
        # # self.controller.PIDDataUpdated.connect(self._pid_data_received)
        #
        # self.dsbRollKP.setValue(self.controller._x_kp)
        # self.dsbRollKI.setValue(self.controller._x_ki)
        # self.dsbRollKD.setValue(self.controller._x_kd)
        # self.dsbRollKP.valueChanged.connect(self.controller._x_kp_changed)
        # self.dsbRollKI.valueChanged.connect(self.controller._x_ki_changed)
        # self.dsbRollKD.valueChanged.connect(self.controller._x_kd_changed)
        #
        # self.sbThrust.setValue(self.controller._thrust)
        # self.sbThrust.valueChanged.connect(self.controller.set_thrust)
        #
        # self.btnStart.clicked.connect(self.controller.control_en_toggle)
        #
        # self.dsbPitchKP.setValue(self.controller._y_kp)
        # self.dsbPitchKI.setValue(self.controller._y_ki)
        # self.dsbPitchKD.setValue(self.controller._y_kd)
        # self.dsbPitchKP.valueChanged.connect(self.controller._y_kp_changed)
        # self.dsbPitchKI.valueChanged.connect(self.controller._y_ki_changed)
        self.controller.PositionUpdated.connect(self._data_received)
        #
        #
        self._plot.set_title('Position')
        color_selector = 0
        self._plot.add_curve('actual.x', self.colors[color_selector % len(self.colors)])
        color_selector += 1
        self._plot.add_curve('actual.y', self.colors[color_selector % len(self.colors)])
        color_selector += 1
        self._plot.add_curve('actual.z', self.colors[color_selector % len(self.colors)])

        #
        # color_selector += 1
        # self._plot.add_curve('target.x', self.colors[color_selector % len(self.colors)])
        # color_selector += 1
        # self._plot.add_curve('target.y', self.colors[color_selector % len(self.colors)])
        # color_selector += 1
        # self._plot.add_curve('target.z', self.colors[color_selector % len(self.colors)])
        #
        # self._plot2.set_title('PID Output')
        # color_selector = 0
        # self._plot2.add_curve('roll', self.colors[color_selector % len(self.colors)])
        # color_selector += 1
        # self._plot2.add_curve('pitch', self.colors[color_selector % len(self.colors)])
        #
        # color_selector += 1
        # self._plot.add_curve('actual.z', self.colors[color_selector % len(self.colors)])
        # color_selector += 1
        # self._plot.add_curve('target.x', self.colors[color_selector % len(self.colors)])
        # color_selector += 1
        # self._plot.add_curve('target.y', self.colors[color_selector % len(self.colors)])
        # color_selector += 1
        # self._plot.add_curve('target.z', self.colors[color_selector % len(self.colors)])

    def _log_data_signal_wrapper(self, ts, data):
        """Wrapper for signal"""
        self._log_data_signal.emit(ts, data)

    def _data_received(self, x, y, z):

        # position = data.split(',')

        pos = {}
        pos['actual.x'] = x
        # pos['target.x'] = 320

        pos['actual.y'] = y
        # pos['target.y'] = 240


        pos['actual.z'] = z
        # pos['target.z'] = 1400
        #
        self._plot.add_data(pos, int(self._plot_time_count))
        self._plot_time_count += ((time.clock() - self.last_time) * 1000)

