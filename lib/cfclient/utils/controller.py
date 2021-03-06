from TiffImagePlugin import _cvt_res

__author__ = 'messierz'

import time
import datetime
import logging
from cflib.utils.callbacks import Caller
from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import QImage, QPixmap

import freenect
import cv2
import cv2.cv as cv
import frame_convert
import numpy as np
import time
import math
import logging
from pid import RollPitchPID, ThrustPID, PitchPID
from cv import Scalar

class Controller(QtCore.QThread):

    PlotUpdated = Caller()
    # PIDDataUpdated = QtCore.pyqtSignal(object, float)
    PositionUpdated = QtCore.pyqtSignal(int, int, int, int, int, int)
    OutputUpdated   = QtCore.pyqtSignal(float, float, float)

    # Sum x, y, z, error count
    ErrorUpdated   = QtCore.pyqtSignal(float, float, float, int)

    ImageUpdated = QtCore.pyqtSignal(object)
    DepthUpdated = QtCore.pyqtSignal(str)

    ThrustUpdated = Caller()

    last_raw_depth = None
    raw_depth_arr = []
    max_raw_depth = 925
    min_raw_depth = 750

    depth_th = 232

    _bg_subtract_state = 1

    _MEAN_DATA_CNT = 10

    _pad_roll = 0
    _pad_pitch = 0
    _pad_yaw = 0
    _pad_thrust = 0

    _last_roll = 0
    _last_agg_roll = False
    _last_pitch = 0
    _last_agg_pitch = False
    _last_thrust = 0
    _last_agg_th = False

    _agg_pid = False

    _mean_arr_z = []
    _mean_arr_y = []
    _mean_arr_x = []

    _target_x = 0
    _target_y = 0
    _target_z = 0

    _emergency_stop = 0

    _obstacles = []

    _all_error = [0.0,0.0,0.0,0.0]
    _error_cnt = 0

    _show_depth_grid = False
    _show_obstacles = False

    _plan_path = []
    _follow_path = []

    def __init__(self, cf):
        QtCore.QThread.__init__(self)

        # Init kinect module
        self.kernel = np.ones((5,5),np.uint8)

        self.thrust = 0

        self.cf = cf

        self.fly_en = False
        self._is_copter_found = False
        self._copter_pos = (0,0,0)

        self.set_x = 150 #320
        self.set_y = 500 #240
        self.set_z = 140

        # Max depth : 208 cm, Min depth : 101 cm
        self.min_depth_in_cm = self._raw_depth_to_cm(self.min_raw_depth)
        self.max_depth_in_cm = self._raw_depth_to_cm(self.max_raw_depth)
        #int(100 * (k3 * math.tan(self.max_raw_depth / k2 + k1)))

        self.r_pid = RollPitchPID(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        # self.r_pid = PID_RP(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        self.p_pid = RollPitchPID(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        # self.p_pid = PitchPID(P=0.05, D=1.0, I=0.00025)
        # self.p_pid = PID_RP(P=0.05, D=1.0, I=0.00025, set_point=0.0)

        self.th_pid = ThrustPID(Kp=150, Ki=200, Kd=50, Input=0, Output=0, Setpoint=self.set_z, ControllerDirection=1)
        self.th_pid.SetOutputLimits(-20000, 20000)

        self.calc_pitch = 0
        self.calc_roll = 0

        self.LIMIT = 20

        self.trim_roll = 0
        self.trim_pitch = 0

        self._is_logging = False
        # EXPR_CNT = 2
        # POINT_CNT = 2
        # FILENAME = 'Expr2_A_to_B_{}_Points_'.format(POINT_CNT)
        FILENAME = 'Expr3_Simple_Path_'
        PATH = '/Users/pnrisk/log/'
        if self._is_logging:
            ts = time.time()
            st = FILENAME + datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H.%M.%S')
            self.file = open(PATH + st +'.log','w+')

        self.last_thrust = 0

        # fps = 30
        # capSize = (640,480) # this is the size of my source video
        # fourcc = cv2.cv.CV_FOURCC('m', 'p', '4', 'v') # note the lower case
        # self.writer = cv2.VideoWriter()
        # success = self.writer.open(PATH+st+'.move',fourcc,fps,capSize,True)
        # self._is_start_rec_vid = False

    def set_follow_path(self, path):
        self._follow_path = path

    def set_planning_path(self, path):
        self._plan_path = path

    def _raw_depth_to_cm(self, raw_depth):

        k1 = 1.1863
        k2 = 2842.5
        k3 = 0.1236

        return int(100 * (k3 * math.tan(raw_depth / k2 + k1)))

    def set_auto_fly(self, en):
        logging.info('auto fly : {}'.format(en))

        self.fly_en = en
        self.th_pid.SetMode(en)
        self.r_pid.set_mode(en)
        self.p_pid.set_mode(en)
        self.p_pid.reset()

    def update_thrust_from_pad(self, r, p, y, thrust):

        self._pad_pitch = p
        self._pad_roll = r
        self._pad_yaw = y
        self._pad_thrust = thrust

    def depth_convert(self, depth):
        max_depth_val = 925
        min_depth_val = 750
        np.clip(depth, min_depth_val, max_depth_val, depth)
        # depth >>= 2
        n = 255 / float(max_depth_val - min_depth_val)
        depth = (depth - min_depth_val ) * n
        depth = depth.astype(np.uint8)
        return depth

    def background_subtraction(self):
        self._bg_subtract_state = 1
        pass

    def fg_find(self):
        if self._bg_subtract_state != 4:
            return
        self._bg_subtract_state = 3
        pass



    def change_th(self, value):
        logging.info("depth : {}".format(value))
        self.depth_th = value

        pass

    def find_obstacle(self):
        self._bg_subtract_state = 2
        self._obstacles[:] = []
        logging.info('clear list {}'.format(len(self._obstacles)))

    def get_obstacles(self):
        return self._obstacles

    def show_bg(self):
        if self._bg_subtract_state != 4:
            return

        # depth_image = frame_convert.my_depth_convert(self.raw_bg_data, self.max_raw_depth, self.min_raw_depth)
        raw_bg_data = np.clip(self.raw_bg_data, 0, 2**10 - 1)
        raw_bg_data = raw_bg_data.astype(np.uint8)
        cvRGBImg = cv2.cvtColor(raw_bg_data, cv2.cv.CV_GRAY2RGB)
        self.ImageUpdated.emit(cvRGBImg)

        # cvRGBImg = cv2.cvtColor(depth_image, cv2.cv.CV_GRAY2RGB)

        # self.ImageUpdated.emit(cvRGBImg)
        logging.info("bg show")


    def get_position(self):

        ret_pos = (-1, -1, -1)

        if self._bg_subtract_state == 1:
            ''' Already to process '''
            pass

        # if self._bg_subtract_state == 2:
        #     ''' Build Obstacle '''
        #     # del self._obstacles[:]
        #     self._obstacles[:] = []
        #     logging.info('clear list {}'.format(len(self._obstacles)))

            # self._obstacles = None
            # self._obstacles = []

        # elif self._bg_subtract_state == 4:
        #     return (is_getpos_success, pos_result)
        # elif self._bg_subtract_state == 0:
        #     # Not initial bg subtract
        #     return (is_getpos_success, pos_result)
        # else:
        #     # Already to process

        try:
            raw_depth = freenect.sync_get_depth()[0]
            # raw_rgb = freenect.sync_get_video()[0]
        except:
            raw_depth = None
            # raw_rgb = None

        if raw_depth == None:
            return (False, ret_pos)

        # show_depth_image = frame_convert.my_depth_convert(np.copy(raw_depth))
        # show_depth_image = frame_convert.video_cv(raw_rgb)
        # if self._is_start_rec_vid:
        #     raw_rgb = raw_rgb[:, :, ::-1]  # RGB -> BGR
        #     self.writer.write(raw_rgb)

        # Normal Threshold
        max_raw_depth = 917
        min_raw_depth = 750
        # raw_depth[raw_depth > max_raw_depth] = 0

        if len(self._obstacles) >= 1 and self._bg_subtract_state == 1:
            obs_cnt = len(self._obstacles)
            for i in xrange(obs_cnt):
                x,y,w,h,z = self._obstacles[i]
                img_obstacle = raw_depth[y: y+h, x: x+w]
                img_obstacle[img_obstacle > z] = 920
                # img_depth[y: y+h, x: x+w] = img_obstacle
                # logging.info('obstacle {} (x,y,w,h) : ({},{},{},{}) z : {}'.format(i+1,x,y,w,h,z))

        img_depth = np.copy(raw_depth)

        fg_filter = img_depth <= max_raw_depth
        img_depth[img_depth > max_raw_depth] = 0
        img_depth[img_depth < min_raw_depth] = 0



        # hist, bin_edges = np.histogram(raw_depth, bins=np.arange(1023))
        # for i in range(len(hist)):
        #     if hist[i] <= 0: continue
        #     logging.info("{} : {}".format(bin_edges[i], hist[i]))

        img_depth[fg_filter] = (max_raw_depth - img_depth[fg_filter]) * 1.5
        img_depth = img_depth.astype(np.uint8)

        ret, th_img = cv2.threshold( img_depth, 1, 255, cv2.THRESH_BINARY_INV )

        # cvRGBImg = cv2.cvtColor(raw_rgb, cv2.cv.CV_GRAY2RGB)
        # cvRGBImg = raw_rgb
        cvRGBImg = cv2.cvtColor(th_img, cv2.cv.CV_GRAY2RGB)

        ''' Old threshold method '''
        # depth_image = frame_convert.my_depth_convert(raw_depth, self.max_raw_depth, self.min_raw_depth)
        # ret, th_img = cv2.threshold( depth_image, self.depth_th, 255, cv2.THRESH_BINARY )

        EMIT_IMAGE = True

        # Find Blob
        contours, hierarchy = cv2.findContours(th_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        copter_count = 0

        for h, cnt in enumerate(contours):

            area = cv2.contourArea(cnt)
            # logging.info("area : {}".format(area))
            ''' Copter area '''
            if area > 300 and area < 1800:

                copter_count += 1

                ''' Break when copter > 1 '''
                if copter_count >= 2: break

                # moments = cv2.moments(cnt)
                ''' Get bounding box '''
                x,y,w,h = cv2.boundingRect(cnt)

                copter_rect = raw_depth[y: y+h, x: x+w]
                copter_rect[copter_rect > max_raw_depth] = 0
                max_freq_depth = 0
                _max_freq_depth = 0

                hist, bin_edges = np.histogram(copter_rect, bins=np.arange(1023))

                for i in xrange(min_raw_depth, len(hist)):
                    if hist[i] <= 0: continue
                    # print '{}  hist {}  bin {}'.format(i, hist[i], bin_edges[i])
                    if hist[i] > _max_freq_depth:
                        _max_freq_depth = hist[i]
                        max_freq_depth = bin_edges[i]

                # self._mean_arr_x.append(x+(w/2))
                # self._mean_arr_y.append(y+(h/2))
                self._mean_arr_z.append(max_freq_depth)

                mean_x = x+(w/2)
                mean_y = y+(h/2)
                mean_z = np.mean(self._mean_arr_z)

                mean_z = int(self._raw_depth_to_cm(mean_z))

                ret_pos = (mean_x, mean_y, mean_z)
                # logging.info('{}'.format(ret_pos))

                ''' Remove data in array '''
                if len(self._mean_arr_z) > 3:
                    self._mean_arr_z.pop(0)

                # if len(self._mean_arr_x) > 3:
                #     self._mean_arr_x.pop(0)

                # if len(self._mean_arr_y) > 3:
                #     self._mean_arr_y.pop(0)


                if EMIT_IMAGE:
                    self._show_obstacles = True

                    ''' Draw obstacles '''
                    if self._show_obstacles and len(self._obstacles) >= 1:
                        obs_cnt = len(self._obstacles)
                        for i in xrange(obs_cnt):
                            _x,_y,_w,_h,_z = self._obstacles[i]
                            _gray_val = ((self.max_depth_in_cm - self._raw_depth_to_cm(_z)) * 192) / (self.max_depth_in_cm - self.min_depth_in_cm)
                            cv2.rectangle(cvRGBImg,(_x,_y), (_x+_w,_y+_h), Scalar(_gray_val,_gray_val,_gray_val), -1)

                    ''' Draw height grid '''
                    self._show_depth_grid = True
                    if self._show_depth_grid:
                        if self._show_obstacles and len(self._obstacles) >= 1:
                            # mean_z = self._raw_depth_to_cm(_z)
                            __z = (self.pz_to_z(1))
                        else :
                            __z = mean_z

                        for i in xrange(1, 10):
                            # y = int((i * ((mean_z * -0.0341) + 9.426) * 10))
                            __y = int((i * ((__z * -0.0341) + 9.426) * 10) + 240)
                            if __y >= 0 and __y < 480:
                                cv2.line(cvRGBImg, (0,__y), (639,__y), (0,255,0), 1)
                            #
                            __y = int((-i * ((__z * -0.0341) + 9.426) * 10) + 240)
                            if __y >= 0 and __y < 480:
                                cv2.line(cvRGBImg, (0,__y), (639,__y), (255,0,0), 1)

                        for i in xrange(1, 15):
                            # x1 = int((i * ((mean_z * -0.0322) + 8.9439) * 10))
                            __x = int((i * ((__z * -0.0322) + 8.9439) * 10) + 320)
                            # __x = x1
                            if __x >= 0 and __x < 640:
                                cv2.line(cvRGBImg, (__x,0), (__x,479), (0,255,0), 1)
                            #
                            __x = int(((-1 * i) * (((__z * -0.0322) + 8.9439) * 10)) + 320)
                            # x = x2
                            if __x >= 0 and __x < 640:
                                cv2.line(cvRGBImg, (__x,0), (__x,479), (255,0,0), 1)




                    ''' Draw path '''
                    if self._plan_path is not None and len(self._plan_path) > 1:
                        for i in xrange(len(self._plan_path) - 1):

                            # plan_z = self.pz_to_z()
                            pz1 = self._plan_path[i][2]

                            plan_x = self.px_to_x(self._plan_path[i][0], self._plan_path[i][2])
                            plan_y = self.py_to_y(self._plan_path[i][1], self._plan_path[i][2])

                            # plan_z = self.pz_to_z(self._plan_path[i+1].z)
                            if pz1 == 1 : color = (128,0,0)
                            elif pz1 == 2 : color = (139,0,0)
                            elif pz1 == 3 : color = (165,42,42)
                            elif pz1 == 4 : color = (178,34,34)
                            elif pz1 == 5 : color = (220,20,60)
                            elif pz1 == 6 : color = (255,0,0)
                            elif pz1 == 7 : color = (255,99,71)
                            elif pz1 == 8 : color = (255,127,80)
                            elif pz1 == 9 : color = (255,165,0)
                            else : color = (255,215,0)

                            plan_x_1 = self.px_to_x(self._plan_path[i+1][0], self._plan_path[i+1][2])
                            plan_y_1 = self.py_to_y(self._plan_path[i+1][1], self._plan_path[i+1][2])
                            #int((self._plan_path[i+1].y * ((plan_z * -0.0341) + 9.426) * 10))

                            # logging.info('start {}, {}  end {}, {}'.format(plan_x, plan_y, plan_x_1, plan_y_1))
                            # img_x = int((plan_x * ((mean_z * -0.0322) + 8.9439) * 10))
                            cv2.line(cvRGBImg, (plan_x, plan_y), (plan_x_1, plan_y_1), color, 5)

                    elif self._follow_path is not None and len(self._follow_path) > 1:
                        for i in xrange(len(self._follow_path) - 1):

                            # plan_z = self.pz_to_z()
                            pz1 = self.z_to_pz(self._follow_path[i][2])

                            plan_x = self._follow_path[i][0]
                            plan_y = self._follow_path[i][1]

                            if pz1 == 1 : color = (128,0,0)
                            elif pz1 == 2 : color = (139,0,0)
                            elif pz1 == 3 : color = (165,42,42)
                            elif pz1 == 4 : color = (178,34,34)
                            elif pz1 == 5 : color = (220,20,60)
                            elif pz1 == 6 : color = (255,0,0)
                            elif pz1 == 7 : color = (255,99,71)
                            elif pz1 == 8 : color = (255,127,80)
                            elif pz1 == 9 : color = (255,165,0)
                            else : color = (255,215,0)

                            plan_x_1 = self._follow_path[i+1][0]
                            plan_y_1 = self._follow_path[i+1][1]

                            cv2.line(cvRGBImg, (plan_x, plan_y), (plan_x_1, plan_y_1), color, 5)

                    ''' Draw bounding rect '''
                    cv2.rectangle(cvRGBImg,(x,y), (x+w,y+h), Scalar(255,255,255), -1)
                    cv2.rectangle(cvRGBImg,(x,y), (x+w,y+h), Scalar(255,0,0), 1)

                    ''' Draw position '''
                    cv2.circle(cvRGBImg,(mean_x, mean_y), 4, (0,0,255), -1)
                    str_out = '{}, {}, {}'.format(mean_x, mean_y, mean_z)
                    cv2.putText(cvRGBImg, str_out, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))



                    ''' Draw target '''
                    LINE_SIZE = 1
                    cv2.line(cvRGBImg, (0,self.set_y), (639,self.set_y), (0,0,255), LINE_SIZE)
                    cv2.line(cvRGBImg, (self.set_x,0), (self.set_x,479), (0,0,255), LINE_SIZE)

            elif self._bg_subtract_state == 2 and area >= 1800 and area < 50000:
                ''' Build Obstacle '''
                x,y,w,h = cv2.boundingRect(cnt)
                padding_value = 25
                x = x-padding_value if x-padding_value >= 0 else 0
                y = y-padding_value if y-padding_value >= 0 else 0
                w = w+(padding_value + padding_value) if x+w+(padding_value + padding_value) < 640 else 640
                h = h+(padding_value + padding_value) if y+h+(padding_value + padding_value) < 480 else 480

                obstacle_rect = raw_depth[y: y+h, x: x+w]
                obstacle_rect[obstacle_rect > max_raw_depth] = 0

                hist, bin_edges = np.histogram(obstacle_rect, bins=np.arange(1023))

                min_depth_cnt = max_raw_depth
                min_depth_val = 0

                for i in xrange(min_raw_depth, len(hist)):
                    if hist[i] <= 0: continue
                    # logging.info('{}  hist {}  bin {}'.format(i, hist[i], bin_edges[i]))
                    if i < min_depth_cnt:
                        min_depth_cnt = i
                        # min_depth_val = bin_edges[i]

                min_depth_val = min_depth_cnt - 5
                min_depth_val = 882

                self._obstacles.append([x,y,w,h,min_depth_val])

                logging.info('obstacle {} area : {} (x,y,w,h) : ({},{},{},{}) z : {}'.format(len(self._obstacles),area,x,y,w,h,min_depth_val))

                if EMIT_IMAGE:
                    ''' Draw bounding rect '''
                    cv2.rectangle(cvRGBImg,(x,y), (x+w,y+h), Scalar(255,0,0), 1)

        if self._bg_subtract_state == 2:
            self._bg_subtract_state = 1

        if EMIT_IMAGE:
            self.ImageUpdated.emit(cvRGBImg)

        # Depth Filter
        if ret_pos is not None and (ret_pos[2] > self.max_depth_in_cm or ret_pos[2] < self.min_depth_in_cm):
            return (False, ret_pos)

        if copter_count == 1:
            return (True, ret_pos)

        return (False, ret_pos)

    def _y_to_cm(self,y,z):
        return (y)/(((z*(-0.0341))+9.4260))
    def _x_to_cm(self,x,z):
        return (x)/(((z*(-0.0322))+8.9439))

    ''' Depth X to Plan X '''
    def x_to_px(self, x, z, ceil=False):

        return int(math.ceil(self._x_to_cm(x-320, z) / 10.0)) + 12

        # pz = self.z_to_pz(z)
        # is_ceil = True if x >= 320 else False
        # if is_ceil: px = int(math.ceil(self._x_to_cm(x-320, z) / 10.0))
        # else: px = int(math.floor(self._x_to_cm(x-320, z) / 10.0))
        # return px + 12

        # if pz == 1: px = px + 12
        # else: return int(math.floor(self._x_to_cm(x, z) / 10.0))

    ''' Depth Y to Plan Y '''
    def y_to_py(self, y, z, ceil=False):

        return int(math.ceil(self._y_to_cm(y-240, z) / 10.0)) + 10

        # is_ceil = True if y >= 240 else False
        # if is_ceil: py = int(math.ceil(self._y_to_cm(y-240, z) / 10.0))
        # else: py = int(math.floor(self._y_to_cm(y-240, z) / 10.0))
        # return py + 10

        # if ceil: return int(math.ceil(self._y_to_cm(y, z) / 10.0))
        # else: return int(math.floor(self._y_to_cm(y, z) / 10.0))

    ''' Plan X to Depth X '''
    def px_to_x(self, px, pz, ceil=False):

        x = int(math.ceil((px - 12) * ((self.pz_to_z(pz) * -0.0322) + 8.9439) * 10) + 320)
        x = 639 if x >= 640 else x
        x = 0 if x < 0 else x
        return x
        # if ceil: return int(math.ceil(px * ((self.pz_to_z(pz) * -0.0322) + 8.9439) * 10))
        # else: return int(math.floor(px * ((self.pz_to_z(pz) * -0.0322) + 8.9439) * 10))

        # return int((px * ((self.pz_to_z(pz) * -0.0322) + 8.9439) * 10))

    ''' Plan Y to Depth Y '''
    def py_to_y(self, py, pz, ceil=False):

        y = int(math.ceil((py - 10) * ((self.pz_to_z(pz) * -0.0341) + 9.426) * 10) + 240)
        y = 479 if y > 479 else y
        y = 0 if y < 0 else y

        return y

        # if ceil: return int(math.ceil(py * ((self.pz_to_z(pz) * -0.0341) + 9.426) * 10))
        # else: return int(math.floor(py * ((self.pz_to_z(pz) * -0.0341) + 9.426) * 10))

        # return int((py * ((self.pz_to_z(pz) * -0.0341) + 9.426) * 10))

    ''' Plan Z to Depth Z '''
    def pz_to_z(self, plan_z):
        return int(200 - (plan_z * 10))

    ''' Depth Z to Plan Z '''
    def z_to_pz(self, z):
        plan_z = 200 - z
        plan_z = 0 if plan_z < 0 else plan_z
        plan_z = 100 if plan_z > 100 else plan_z
        return int(math.ceil(plan_z / 10.0))


    def slot_reset_error(self):
        self._all_error = [0,0,0]
        self._error_cnt = 0
    def slot_set_agg_pid(self, agg = False):
        self._agg_pid = agg

    def set_target_x(self, x):
        self.set_x = x

    def set_target_y(self, y):
        self.set_y = y

    def set_target_z(self, z):
        self.set_z = z
        self.th_pid.SetTargetPoint(z)

    def copter_found(self):
        return self._is_copter_found

    def get_copter_position(self):
        if self._is_copter_found:
            return self._copter_pos
        else:
            return (0,0,0)

    def run(self):

        d_time_count = 0
        n_frame_count = 0

        # self.p_pid.tuning(0.125, 2.1, 0.0005)
        # self.p_pid.tuning(0.05, 0.05, 0.00015)
        self.p_pid.tuning(0.05, 1.0, 0.0001)
        self.r_pid.tuning(0.05, 1.0, 0.0001)
        # self.p_pid.set_integrator_limit(-20000, 20000)

        while True:

            start_time = time.clock()

            found, pos = self.get_position()

            self._is_copter_found = found
            if found:
                ''' Export position data '''
                self.PositionUpdated.emit(pos[0], pos[1], pos[2], self.set_x, self.set_y, self.set_z)
                self._copter_pos = pos

            self._is_start_rec_vid = self.fly_en

            if self.fly_en:

                if found:
                    # Reset emergency count
                    self._emergency_stop = 0

                    diff_time = (time.clock() - start_time) * 1000

                    if self._is_logging:
                        self.file.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(diff_time, pos[0], pos[1], pos[2], self.set_x, self.set_y, self.set_z))

                    self._all_error[0] += math.fabs(pos[0] - self.set_x)
                    self._all_error[1] += math.fabs(pos[1] - self.set_y)
                    self._all_error[2] += math.fabs(pos[2] - self.set_z)
                    self._error_cnt += 1
                    self.ErrorUpdated.emit(self._all_error[0], self._all_error[1], self._all_error[2], self._error_cnt)

                    now_time = time.clock()

                    # self._copter_pos = pos
                    actual_y_cm = self._y_to_cm(pos[1], pos[2])
                    target_y_cm = self._y_to_cm(self.set_y, pos[2])

                    is_agg_pitch = False if math.fabs(actual_y_cm - target_y_cm) < 10 else True
                    if not self._agg_pid: is_agg_pitch = False
                    is_agg_pitch = False

                    if self._last_agg_pitch != is_agg_pitch:
                        self._last_agg_pitch = is_agg_pitch

                        if is_agg_pitch:
                            # self.p_pid.tuning(0.15, 2.1, 0) # 0.00035)
                            self.p_pid.tuning(0.05, 1.0, 0.0001)

                        else:
                            self.p_pid.tuning(0.05, 1.0, 0.0001)

                        logging.info('pitch pid change agg : {}'.format(is_agg_pitch))

                    actual_x_cm = self._x_to_cm(pos[0], pos[2])
                    target_x_cm = self._x_to_cm(self.set_x, pos[2])

                    is_agg_roll = False if math.fabs(actual_x_cm - target_x_cm) < 10 else True
                    if not self._agg_pid: is_agg_roll = False
                    is_agg_roll = False

                    if self._last_agg_roll != is_agg_roll:
                        self._last_agg_roll = is_agg_roll

                        if is_agg_roll:
                            # self.r_pid.tuning(0.085, 1.5, 0.00025)
                            self.r_pid.tuning(0.05, 1.0, 0.00015)

                        else:
                            # self.r_pid.tuning(0.025, 1.0, 0.00015)
                            self.r_pid.tuning(0.05, 1.0, 0.00015)


                        logging.info('roll pid change agg : {}'.format(is_agg_roll))

                    actual_z_cm = pos[2]
                    target_z_cm = self.set_z

                    is_agg_th = False if math.fabs(actual_z_cm - target_z_cm) < 10 else True
                    if not self._agg_pid: is_agg_th = False
                    is_agg_th = False

                    if self._last_agg_th != is_agg_th:
                        self._last_agg_th = is_agg_th

                        if is_agg_th:
                            self.th_pid.SetTuning(300, 200, 50)
                        else:
                            self.th_pid.SetTuning(150, 200, 50)

                        logging.info('roll pid change agg : {}'.format(is_agg_th))

                    # X - Roll calculate
                    if self.r_pid.update(self.set_x - pos[0]):
                        roll = self.r_pid.get_output()
                    else:
                        roll = self._last_roll

                    # Y - Pitch calculate
                    if self.p_pid.update(self.set_y - pos[1]):
                        pitch = self.p_pid.get_output()
                    else:
                        pitch = self._last_pitch

                    if self.th_pid.Compute(pos[2]):
                        thrust = self.th_pid.GetOutput()
                    else:
                        thrust = self._last_thrust

                    roll_sp = -roll
                    pitch_sp = pitch
                    thrust_sp = (thrust) + 40000

                    self._last_thrust = thrust
                    self._last_roll = roll
                    self._last_pitch = pitch
                    # self._last_pitch = pitch

                    if roll_sp > self.LIMIT: roll_sp = self.LIMIT
                    elif roll_sp < -self.LIMIT: roll_sp = -self.LIMIT

                    if pitch_sp > self.LIMIT: pitch_sp = self.LIMIT
                    elif pitch_sp < -self.LIMIT: pitch_sp = -self.LIMIT

                    if thrust_sp > 63000: thrust_sp = 63000
                    elif thrust_sp < 0: thrust_sp = 0

                    self.calc_pitch = pitch_sp
                    self.calc_roll = roll_sp
                    self.thrust = thrust_sp

                    self.OutputUpdated.emit(self.calc_roll, self.calc_pitch, self.thrust)

                    final_roll = self.calc_roll + self.trim_roll
                    final_pitch = self.calc_pitch + self.trim_pitch

                    # Automatic copter control
                    self.cf.commander.send_setpoint(final_roll, final_pitch, 0, self.thrust)

                else:
                    self._emergency_stop += 45

                if self._emergency_stop > 10:
                    ''' Emergency Stop when copter not found '''
                    self.cf.commander.send_setpoint(0, 0, 0, 0)
                    self.cf.commander.send_setpoint(0, 0, 0, 0)
                    self.cf.commander.send_setpoint(0, 0, 0, 0)
                    self.fly_en = False

            else:
                # Manual copter control
                if self.cf.state == 2:
                    self.cf.commander.send_setpoint(self._pad_roll / 2, self._pad_pitch / 2, 0, self._pad_thrust)

            end_time = time.clock()

            d_time_count += ((end_time - start_time) * 1000)
            n_frame_count += 1

            if n_frame_count > 90:

                # logging.info('avg time : {} ms.'.format(d_time_count / n_frame_count))
                # logging.info("max depth : {} cm,  min depth : {}".format(self.max_depth_in_cm, self.min_depth_in_cm))

                d_time_count = 0
                n_frame_count = 0

            time.sleep(0.0001)

        if self._is_logging:
            self.file.close()

        # self.writer.release()
        # self.writer = None

    def get_image(self):

        raw_depth = freenect.sync_get_depth()[0]

        self.last_raw_depth = np.copy(raw_depth)

        depth_image = frame_convert.my_depth_convert(raw_depth, self.max_raw_depth, self.min_raw_depth)


        # raw_rgb = freenect.sync_get_video()[0]

        cvRGBImg = cv2.cvtColor(depth_image, cv2.cv.CV_GRAY2RGB)



        self.ImageUpdated.emit(cvRGBImg)