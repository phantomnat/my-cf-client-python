__author__ = 'pnrisk'


from PyQt4 import QtCore, QtGui
import time
import logging

class Planner(QtCore.QThread):

    def __init__(self, controller):
        QtCore.QThread.__init__(self)

        self.controller = controller

        self._work_state = 0
        self._state = 0

    def task4(self):

        self.controller.slot_reset_error()
        self.controller.slot_set_agg_pid(True)

        # Get obstacle
        obs = self.controller.get_obstacles()


    def task3(self):

        self.controller.slot_reset_error()
        self.controller.slot_set_agg_pid(True)

        self.start_point = (550, 220, 150)
        self.controller.set_target_x(self.start_point[0])
        self.controller.set_target_y(self.start_point[1])
        self.controller.set_target_z(self.start_point[2])
        self.controller.set_auto_fly(True)

        self.target_points = []
        # self.target_points.append((550, 220, 130))
        self.target_points.append((550, 220, 150))
        self.target_points.append((500, 220, 140))
        self.target_points.append((200, 220, 140))
        self.target_points.append((140, 220, 160))

        self._landing_pos = (140, 220, 190)

        self._work_state = 4

    def take_off(self):
        self._work_state = 1
        self._state = 0

    def new_landing(self):
        self._work_state = 9
        self._state = 0

    def landing(self):
        self._work_state = 2
        self._state = 0

    def task_a2b(self):
        self._work_state = 3

        self.controller.slot_reset_error()
        self.controller.slot_set_agg_pid(True)

        self.start_point = (550, 220, 160)
        self.controller.set_target_x(self.start_point[0])
        self.controller.set_target_y(self.start_point[1])
        self.controller.set_target_z(self.start_point[2])
        self.controller.set_auto_fly(True)

        self.target_points = []
        self.target_points.append((550, 220, 160))
        self.target_points.append((500, 220, 120))
        self.target_points.append((260, 220, 120))
        self.target_points.append((140, 220, 160))

        self._landing_pos = (140, 220, 190)

        # self.target_points.append((180, 150, 140))
        #
        # self.target_points.append((180, 391, 140))
        # self.target_points.append((480, 391, 140))
        # self.target_points.append((480, 150, 140))
        # self.target_points.append((180, 150, 140))
        #
        # self.target_points.append((180, 391, 140))
        # self.target_points.append((480, 391, 140))
        # self.target_points.append((480, 150, 140))
        # self.target_points.append((180, 150, 140))
        #
        # self.target_points.append((180, 391, 140))
        # self.target_points.append((480, 391, 140))
        # self.target_points.append((480, 150, 140))
        # self.target_points.append((180, 150, 140))
        self._state = 0

    def emergency_stop(self):
        pass

    def _copter_pos_around(self, copter_pos, target_pos, radius = 20):
        # chk_x = copter_pos[0] > (target_pos[0] - radius) and copter_pos[0] < (target_pos[0] + radius)
        # chk_y = copter_pos[1] > (target_pos[1] - radius) and copter_pos[1] < (target_pos[1] + radius)
        # chk_z = copter_pos[2] > (target_pos[2] - radius) and copter_pos[2] < (target_pos[2] + radius)
        # if not chk_x: logging.info('x error : {}  {}'.format(copter_pos[0], target_pos[0]))
        # if not chk_y: logging.info('y error : {}  {}'.format(copter_pos[1], target_pos[1]))
        # if not chk_z: logging.info('z error : {}  {}'.format(copter_pos[2], target_pos[2]))
        return copter_pos[0] > (target_pos[0] - radius) and copter_pos[0] < (target_pos[0] + radius) \
                   and copter_pos[1] > (target_pos[1] - radius) and copter_pos[1] < (target_pos[1] + radius) \
                   and copter_pos[2] > (target_pos[2] - radius) and copter_pos[2] < (target_pos[2] + radius)

    def run(self):

        # if new command
        while True:

            # break

            if self._work_state == 0:
                # IDLE
                pass

            elif self._work_state == 1:
                # Auto Takeoff
                self.controller.set_auto_fly(True)
                self._work_state = 0


            elif self._work_state == 2:
                # Auto landing
                landing_pos = (320, 240)
                if self._state == 0:
                    self.controller.set_target_x(landing_pos[0])
                    self.controller.set_target_y(landing_pos[1])
                    self.controller.set_target_z(150)
                    self._state =1

                elif self._state == 1:
                    ''' Check State '''
                    pos = self.controller.get_copter_position()
                    if self._copter_pos_around(pos, (landing_pos[0], landing_pos[1], 150)):
                        self._state = 2


                elif self._state == 2:
                    self.controller.set_target_z(175)
                    # self._time_cnt = 0
                    # self._last_time = time.clock()
                    self._state = 3

                elif self._state == 3:
                    ''' Check State '''
                    pos = self.controller.get_copter_position()
                    if self._copter_pos_around(pos, (landing_pos[0], landing_pos[1], 175), radius=10):
                        self._state = 4

                elif self._state == 4:
                    self.controller.set_target_z(200)
                    self._time_cnt = 0
                    self._last_time = time.clock()
                    self._state = 5

                elif self._state == 5:
                    ''' Check State '''
                    cur_time = time.clock()
                    self._time_cnt += (cur_time - self._last_time) * 1000
                    self._last_time = cur_time
                    if self._time_cnt > 500:
                        self._state = 6

                elif self._state == 6:
                    self.controller.set_auto_fly(False)
                    self._work_state = 0
                    self._state = 0

            elif self._work_state == 3:
                ''' Task A to B '''

                if self._state == 0:
                    if len(self.target_points) > 0:
                        self.cur_target_points = self.target_points.pop(0)
                        self.controller.set_target_x(self.cur_target_points[0])
                        self.controller.set_target_y(self.cur_target_points[1])
                        self.controller.set_target_z(self.cur_target_points[2])
                        self._state = 1
                    else:
                        ''' Auto landing '''
                        self.landing()

                elif self._state == 1:
                    ''' Check State '''
                    pos = self.controller.get_copter_position()
                    if self._copter_pos_around(pos, self.cur_target_points):
                        self._state = 0


            elif self._work_state == 4:
                ''' Task 3 '''

                if self._state == 0:
                    if len(self.target_points) > 0:
                        self.cur_target_points = self.target_points.pop(0)
                        self.controller.set_target_x(self.cur_target_points[0])
                        self.controller.set_target_y(self.cur_target_points[1])
                        self.controller.set_target_z(self.cur_target_points[2])
                        self._state = 1
                    else:
                        ''' Auto landing '''
                        self.new_landing()

                elif self._state == 1:
                    ''' Check State '''
                    pos = self.controller.get_copter_position()
                    if self._copter_pos_around(pos, self.cur_target_points):
                        self._state = 0


            elif self._work_state == 9:
                ''' Landing '''
                self.controller.slot_set_agg_pid(True)
                lp = self._landing_pos
                if self._state == 0:
                    self.controller.set_target_x(lp[0])
                    self.controller.set_target_y(lp[1])
                    self.controller.set_target_z(lp[2] - 20)
                    self._state =1

                elif self._state == 1:
                    ''' Check State '''
                    pos = self.controller.get_copter_position()
                    if self._copter_pos_around(pos, (lp[0], lp[1], lp[2] - 20)):
                        self._state = 2


                elif self._state == 2:
                    self.controller.set_target_z(lp[2])
                    self._time_cnt = 0
                    self._last_time = time.clock()
                    self._state = 3

                elif self._state == 3:
                    ''' Check State '''
                    cur_time = time.clock()
                    self._time_cnt += (cur_time - self._last_time) * 1000
                    self._last_time = cur_time
                    if self._time_cnt > 500:
                        self._state = 4

                elif self._state == 4:
                    self.controller.set_auto_fly(False)
                    self._work_state = 0
                    self._state = 0

            time.sleep(0.0001)