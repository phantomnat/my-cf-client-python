__author__ = 'pnrisk'


from PyQt4 import QtCore, QtGui
import time
import logging

class Planner(QtCore.QThread):

    def __init__(self, controller):
        QtCore.QThread.__init__(self)

        self.state = 0

        self.controller = controller

    def run(self):

        # if new command
        while True:

            if self.controller.copter_found():

                if self.state == 0:
                    self.state = 1
                    logging.info('Planner : state = 1')

                elif self.state == 1:
                    self.controller.set_target_x(200)
                    self.controller.set_target_y(200)
                    self.controller.set_target_z(150)
                    self.state = 2
                    logging.info('Planner : state = 2')

                elif self.state == 2:
                    pos = self.controller.get_copter_position()
                    if pos[0] > 180 and pos[0] < 220 and pos[1] > 180 and pos[1] < 220 and pos[2] > 140 and pos[2] < 160:
                        logging.info('Planner : state = 3')
                        self.state = 3
                elif self.state == 3:
                    self.controller.set_target_x(400)
                    self.controller.set_target_y(200)
                    self.controller.set_target_z(150)
                    self.state = 4
                    logging.info('Planner : state = 4')
                elif self.state == 4:
                    pos = self.controller.get_copter_position()
                    if pos[0] > 380 and pos[0] < 420 and pos[1] > 180 and pos[1] < 220 and pos[2] > 140 and pos[2] < 160:
                        logging.info('Planner : state = 5')
                        self.state = 4

                elif self.state == 5:
                    self.controller.set_target_z(200)
                pass
            else:
                self.state = 0

            time.sleep(0.0001)