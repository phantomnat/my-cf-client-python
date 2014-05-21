__author__ = 'pnrisk'


from PyQt4 import QtCore, QtGui
import time
import logging
from Point3D import Point3D
import math
import PathPlanning
from PathPlanning import PathFinding

class Planner(QtCore.QThread):

    def __init__(self, controller):
        QtCore.QThread.__init__(self)

        self.controller = controller

        self._work_state = 0
        self._state = 0
        self.target_points = []


    def AddObstacle(self, plan_map, pt_key):
        if pt_key in plan_map:
            # remove data from dict
            del plan_map[pt_key]

            # search in neighbor
            pt_x, pt_y, pt_z = pt_key

            for i in xrange(-1, 2):
                for j in xrange(-1, 2):
                    for k in xrange(-1, 2):
                        src_key = (pt_x + i, pt_y + j, pt_z + k)
                        # Search in
                        if src_key in plan_map:
                            # Search in list
                            if pt_key in plan_map[src_key]:
                                plan_map[src_key].remove(pt_key)
                                # del plan_map[src_key][plan_map[src_key].index(pt_key)]

    def path_planning(self):

        plan_map = PathPlanning.Graph()

        obs = self.controller.get_obstacles()

        start_pt = (18, 10, 4)
        end_pt = (5, 6, 4)
        obstacle_list = []

        # Build obstacle
        if len(obs) >= 1:
            for i in xrange(len(obs)):
                x,y,w,h,z = obs[i]

                z = self.controller._raw_depth_to_cm(z)
                pz = self.controller.z_to_pz(z) + 1
                # max_pz = pz

                # for j in xrange(start_x, end_x + 1):
                #     for k in xrange(start_y, end_y + 1):
                #         pt_key = (j, y, pz)
                #         self.AddObstacle(plan_map, pt_key)

                while pz > 0:
                    z = self.controller.pz_to_z(pz - 1) if pz > 1 else self.controller.pz_to_z(1)
                    start_x = self.controller.x_to_px(x, z)
                    start_y = self.controller.y_to_py(y, z)
                    end_x = self.controller.x_to_px(x + w, z)
                    end_y = self.controller.y_to_py(y + h, z)

                    logging.info('start : ({}, {})   end : ({}, {})  z : {}'.format(start_x, start_y, end_x, end_y, pz))
                    for j in xrange(start_x, end_x + 1):
                        for k in xrange(start_y, end_y + 1):
                            pt_key = (j, k, pz)
                            # logging.info('({}, {}, {}) is obstacle'.format(j, k, pz))
                            self.AddObstacle(plan_map, pt_key)
                    pz -= 1

        path_finder = PathFinding(plan_map, start_pt, end_pt, (25,19,11))

        self.target_points[:] = []

        result = path_finder.Solve()

        logging.info(result)

        self.controller.set_planning_path(result)

        for pt in result:
            z = self.controller.pz_to_z(pt[2])
            x = self.controller.px_to_x(pt[0], pt[2])
            y = self.controller.py_to_y(pt[1], pt[2])
            self.target_points.append((x,y,z))
            logging.info(' ({},{},{}) '.format(x,y,z))

        # copter_pos = self.controller.get_copter_position()
        # logging.info('copter : {}, {}, {}'.format(copter_pos[0], copter_pos[1], copter_pos[2]))
        # c_px = self.controller.x_to_px(copter_pos[0], copter_pos[2])
        # c_py = self.controller.y_to_py(copter_pos[1], copter_pos[2])
        # c_pz = self.controller.z_to_pz(copter_pos[2])
        # logging.info('copter p: {}, {}, {}'.format(c_px, c_py, c_pz))
        #
        # planning_map = self.Graph()
        #
        # self.start_point = (c_px, c_py, c_pz)
        # self.end_point = (17,7,2)
        #
        # obs = self.controller.get_obstacles()
        #
        # obstacle_list = []
        #
        # # Build obstacle
        # if len(obs) >= 1:
        #     for i in xrange(len(obs)):
        #         x,y,w,h,z = obs[i]
        #
        #         # logging.info('start ({}, {}) end ({}, {})'.format(x, y, x+w, y+h))
        #         z = self.controller._raw_depth_to_cm(z)
        #         pz = self.controller.z_to_pz(z) + 1
        #
        #         obs_pz = self.controller.z_to_pz(z) + 1
        #
        #         while pz > 0:
        #
        #             z = self.controller.pz_to_z(pz)
        #
        #             start_x = self.controller.x_to_px(x, z) # int(math.floor(self.controller._x_to_cm(x, z) / 10.0))
        #             start_y = self.controller.y_to_py(y, z) #int(math.floor(self.controller._x_to_cm(y, z) / 10.0))
        #             end_x = self.controller.x_to_px(x+w, z, True) #int(math.ceil(self.controller._x_to_cm(x+w, z) / 10.0))
        #             end_y = self.controller.y_to_py(y+h, z, True) #int(math.ceil(self.controller._y_to_cm(y+h, z) / 10.0))
        #
        #             for j in xrange(start_x, end_x + 1):
        #                 for k in xrange(start_y, end_y + 1):
        #                     p = Point3D(j, k, obs_pz)
        #                     if p in obstacle_list:
        #                         pass
        #                     else:
        #                         p_found = False
        #                         for l in xrange(len(obstacle_list)):
        #                             if obstacle_list[l].x == p.x and obstacle_list[l].y == p.y:
        #                                 p_found = True
        #                                 if obstacle_list[l].z >= obs_pz:
        #                                     break
        #                                 else:
        #                                     # logging.info('lz change : {} to {}'.format(obstacle_list[l].z, p.z))
        #                                     obstacle_list[l].z = obs_pz
        #                                     break
        #                         if not p_found: obstacle_list.append(p)
        #
        #             logging.info('pz : {}'.format(pz))
        #             pz -= 1
        #
        #         # logging.info('pz : {}'.format(pz))
        #         # pz -= 1
        #
        # # for i in obstacle_list:
        # #     logging.info('{}'.format(i))
        #
        # start = Point3D(point=self.start_point)
        # goal = Point3D(point=self.end_point)
        # #
        # p = PathPlanning(start, goal, 30, 20, 10, obstacle_list)
        # result = p.Solve()
        #
        # for i in result:
        #     logging.info(' ({},{},{}) '.format(i.x, i.y, i.z))
        # # self.target_points = []
        # # self.target_points.append(Point3D(self.controller.x_to_px(550, 160), self.controller.y_to_py(220, 160), self.controller.z_to_pz(160)))
        # # self.target_points.append(Point3D(self.controller.x_to_px(500, 120), self.controller.y_to_py(220, 120), self.controller.z_to_pz(120)))
        # # self.target_points.append(Point3D(self.controller.x_to_px(260, 120), self.controller.y_to_py(220, 120), self.controller.z_to_pz(120)))
        # # self.target_points.append(Point3D(self.controller.x_to_px(140, 160), self.controller.y_to_py(220, 160), self.controller.z_to_pz(160)))
        # # self.target_points.append(Point3D(self.controller.x_to_px(140, 190), self.controller.y_to_py(220, 190), self.controller.z_to_pz(190)))
        # # result = []
        #
        #
        # self.target_points[:] = []
        # for i in result:
        #     z = self.controller.pz_to_z(i.z)
        #     x = self.controller.px_to_x(i.x, i.z, False)
        #     y = self.controller.py_to_y(i.y, i.z, False)
        #     self.target_points.append((x,y,z))
            # logging.info(' ({},{},{}) '.format(x,y,z))


    def task4(self):

        self.controller.slot_reset_error()
        self.controller.slot_set_agg_pid(True)

        # Get obstacle
        if len(self.target_points) >= 2:
            self.start_point = self.target_points[0]
            self._landing_pos = self.target_points.pop()
            self.controller.set_auto_fly(True)
            self._work_state = 5

    def task3(self):

        self.controller.slot_reset_error()
        self.controller.slot_set_agg_pid(True)

        self.start_point = (520, 150, 140)
        self.controller.set_target_x(self.start_point[0])
        self.controller.set_target_y(self.start_point[1])
        self.controller.set_target_z(self.start_point[2])
        self.controller.set_auto_fly(True)

        self.target_points[:] = []
        # self.target_points.append((550, 220, 130))
        # self.target_points.append((520, 120, 150))
        self.target_points.append((520, 150, 140))
        self.target_points.append((390, 120, 140))
        self.target_points.append((225, 120, 140))
        self.target_points.append((150, 150, 140))
        self.target_points.append((150, 225, 150))
        self.target_points.append((150, 330, 150))
        self.target_points.append((225, 330, 150))
        # self.target_points.append((390, 330, 150))
        self.target_points.append((520, 330, 150))
        self.target_points.append((520, 225, 180))

        self._landing_pos = (520, 225, 180)

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

        self.controller.slot_reset_error()
        self.controller.slot_set_agg_pid(True)

        self.start_point = (500, 150, 150)
        self.controller.set_target_x(self.start_point[0])
        self.controller.set_target_y(self.start_point[1])
        self.controller.set_target_z(self.start_point[2])
        self.controller.set_auto_fly(True)

        self.target_points[:] = []

        self.target_points.append((500, 150, 150))
        self.target_points.append((500, 250, 150))
        self.target_points.append((500, 350, 150))
        self.target_points.append((400, 350, 150))
        self.target_points.append((300, 350, 150))
        self.target_points.append((200, 350, 150))
        self.target_points.append((200, 250, 150))
        self.target_points.append((200, 150, 150))
        self.target_points.append((500, 150, 150))
        self.target_points.append((320, 240, 170))

        self._landing_pos = (320, 240, 170)

        result = []

        for pt in self.target_points:
            result.append(pt)

        self.controller.set_follow_path(result)

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
        self._work_state = 3


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
                        self.new_landing()

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

            elif self._work_state == 5:
                ''' Task 4 - Path Planning '''

                if self._state == 0:
                    if len(self.target_points) > 0:
                        self.cur_target_points = self.target_points.pop(0)
                        self.controller.set_target_x(self.cur_target_points[0])
                        self.controller.set_target_y(self.cur_target_points[1])
                        self.controller.set_target_z(self.cur_target_points[2])
                        logging.info('goto ({}, {}, {})'.format(self.cur_target_points[0],self.cur_target_points[1],self.cur_target_points[2]))
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
                    self.controller.set_target_z(lp[2])

                    # if lp[2] < 170:
                    #     self.controller.set_target_z(lp[2] - 20)
                    # else:
                    self._state =1

                elif self._state == 1:
                    ''' Check State '''
                    pos = self.controller.get_copter_position()
                    # if self._copter_pos_around(pos, (lp[0], lp[1], lp[2] - 20)):
                    if self._copter_pos_around(pos, (lp[0], lp[1], lp[2])):
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