__author__ = 'messierz'

import time
import datetime
import logging
from cflib.utils.callbacks import Caller
from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import QImage, QPixmap

import freenect
import cv2
import frame_convert
import numpy as np
import time
import math
import logging
from pid import RollPitchPID, ThrustPID
from cv import Scalar

class Controller(QtCore.QThread):

    PlotUpdated = Caller()
    # PIDDataUpdated = QtCore.pyqtSignal(object, float)
    PositionUpdated = QtCore.pyqtSignal(int, int, int, int, int, int)
    OutputUpdated   = QtCore.pyqtSignal(float, float, float)
    ImageUpdated = QtCore.pyqtSignal(object)
    DepthUpdated = QtCore.pyqtSignal(str)

    ThrustUpdated = Caller()

    last_raw_depth = None
    raw_depth_arr = []
    max_raw_depth = 925
    min_raw_depth = 750

    depth_th = 232

    _bg_subtract_state = 2



    _MEAN_DATA_CNT = 10

    _last_roll = 0
    _last_agg_roll = True
    _last_pitch = 0
    _last_agg_pitch = True
    _last_thrust = 0

    _mean_arr_z = []
    _mean_arr_y = []
    _mean_arr_x = []

    _target_x = 0
    _target_y = 0
    _target_z = 0

    def __init__(self, cf):
        QtCore.QThread.__init__(self)

        # Init kinect module
        self.kernel = np.ones((5,5),np.uint8)

        self.thrust = 0

        self.cf = cf
        self.thrust_pad = 0

        self.fly_en = False
        self._is_copter_found = False
        self._copter_pos = (0,0,0)

        self.set_x = 320
        self.set_y = 240
        self.set_z = 150

        # Max depth : 208 cm, Min depth : 101 cm
        self.min_depth_in_cm = self._raw_depth_to_cm(self.min_raw_depth)
        self.max_depth_in_cm = self._raw_depth_to_cm(self.max_raw_depth)
        #int(100 * (k3 * math.tan(self.max_raw_depth / k2 + k1)))

        self.r_pid = RollPitchPID(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        # self.r_pid = PID_RP(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        self.p_pid = RollPitchPID(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        # self.p_pid = PID_RP(P=0.05, D=1.0, I=0.00025, set_point=0.0)

        self.th_pid = ThrustPID(Kp=100, Ki=200, Kd=50, Input=0, Output=0, Setpoint=150, ControllerDirection=1)
        self.th_pid.SetOutputLimits(-20000, 20000)

        self.np_pid = ThrustPID(Kp=0.05, Kd=1, Ki=0.0002, Setpoint=0, Input=0, Output=0, ControllerDirection=0)
        self.np_pid.SetOutputLimits(-20, 20)


        self.np_pid.SetTargetPoint(self.set_y)

        self.calc_pitch = 0
        self.calc_roll = 0

        self.LIMIT = 20

        self.trim_roll = 0
        self.trim_pitch = 0



        self._is_logging = False
        EXPR_CNT = 3
        FILENAME = 'Hold_Pos_{0:02d}_NoAir_NoAgg_'.format(EXPR_CNT)
        if self._is_logging:
            ts = time.time()
            st = FILENAME + datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H.%M.%S')
            self.file = open('/home/messierz/log/'+ st +'.log','w+')

        self.last_thrust = 0

    def _raw_depth_to_cm(self, raw_depth):

        k1 = 1.1863
        k2 = 2842.5
        k3 = 0.1236

        return int(100 * (k3 * math.tan(raw_depth / k2 + k1)))

    def set_auto_fly(self, en):
        logging.info('auto fly : {}'.format(en))

        self.fly_en = en
        # self.th_pid.SetMode(en)
        self.np_pid.SetMode(en)
        self.th_pid.SetMode(en)
        self.r_pid.set_mode(en)
        self.p_pid.set_mode(en)


    def update_thrust(self, r, p, y, thrust):
        # if thrust > 50000:
        #     self.fly_en = True
        # else:
        #     self.fly_en = False
        #
        self.thrust_pad = thrust

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
        global last_depth_data

        EMIT_IMAGE = False

        is_getpos_success = False
        pos_result = (-1, -1, -1)

        if self._bg_subtract_state == 1:
            # Initial bg subtract
            logging.info("bg init.")
            self.raw_bg_data = np.copy(freenect.sync_get_depth()[0]) + 10

            filter = self.raw_bg_data > 923
            self.raw_bg_data[filter] = 0

            raw_bg_data = np.clip(self.raw_bg_data, 0, 2**10 - 1)

            # depth_image = frame_convert.my_depth_convert(raw_bg_data, 940, 920)

            # 914 is floor
            hist, bin_edges = np.histogram(raw_bg_data, bins=np.arange(1023))
            for i in range(len(hist)):
                if hist[i] <= 0: continue
                logging.info("{} : {}".format(bin_edges[i], hist[i]))

            raw_bg_data >>= 2
            raw_bg_data = raw_bg_data.astype(np.uint8)
            cvRGBImg = cv2.cvtColor(raw_bg_data, cv2.cv.CV_GRAY2RGB)

            # self.ImageUpdated.emit(cvRGBImg)

            self._bg_subtract_state = 2

            return (is_getpos_success, pos_result)

        elif self._bg_subtract_state == 4:

            return (is_getpos_success, pos_result)

        elif self._bg_subtract_state == 0:
            # Not initial bg subtract
            return (is_getpos_success, pos_result)

        else:
            # Already to process
            pass

        # raw_depth = freenect.sync_get_depth()[0] - self.raw_bg_depth

        raw_depth = freenect.sync_get_depth()[0]
        # raw_rgb = freenect.sync_get_video()[0]

        # raw_depth = cv2.morphologyEx(raw_depth, cv2.MORPH_OPEN, kernel)
        # raw_depth = cv2.medianBlur(raw_depth, 3)
        # raw_depth = np.fabs(np.subtract(freenect.sync_get_depth()[0], self.raw_bg_depth))

        # raw_depth = cv2.GaussianBlur(raw_depth,(5,5),0)

        # Bg Substract
        # filter = raw_depth > 920

        max_raw_depth = 916

        img_depth = np.copy(raw_depth)

        fg_filter = img_depth <= max_raw_depth
        img_depth[img_depth > max_raw_depth] = 0
        img_depth[img_depth < 750] = 0

        # hist, bin_edges = np.histogram(raw_depth, bins=np.arange(1023))
        # for i in range(len(hist)):
        #     if hist[i] <= 0: continue
        #     logging.info("{} : {}".format(bin_edges[i], hist[i]))

        img_depth[fg_filter] = (max_raw_depth - img_depth[fg_filter]) * 1.5
        img_depth = img_depth.astype(np.uint8)

        ret, th_img = cv2.threshold( img_depth, 1, 255, cv2.THRESH_BINARY_INV )

        # raw_fg_depth = raw_depth  # np.subtract(self.raw_bg_data, raw_depth)
        # np.clip(raw_depth, 0, 920, out=raw_depth)
        # n = 255.0 / float(920)
        # raw_depth = (raw_depth) * n

        # raw_depth >>= 2
        # raw_fg_depth = raw_depth.astype(np.uint8)
        # ret, th_img = cv2.threshold( raw_fg_depth, 5, 255, cv2.THRESH_BINARY )




        # depth_image = frame_convert.my_depth_convert(raw_depth, 940, 920)


        # Normal Threshold
        # depth_image = frame_convert.my_depth_convert(raw_depth, self.max_raw_depth, self.min_raw_depth)
        # ret, th_img = cv2.threshold( depth_image, self.depth_th, 255, cv2.THRESH_BINARY )

        cvRGBImg = cv2.cvtColor(th_img, cv2.cv.CV_GRAY2RGB)

        # cv.CvtColor(difference, grey_image, cv.CV_RGB2GRAY)
        # cv2.dilate(depth_image, )



        # cv.Dilate(grey_image, grey_image, None, 18)
        # cv.Erode(grey_image, grey_image, None, 10)
        # cv2.morphologyEx(depth_image, cv2.MORPH_OPEN, kernel, dst = depth_image)
        # cv2.dilate(depth_image, kernel, dst = depth_image, iterations = 1)

        # storage = cv.CreateMemStorage(0)
        # contour = cv.FindContours(grey_image, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
        # points = []
        # cv.Threshold(depth_image, depth_image, 70, 255, cv.CV_THRESH_BINARY)


        # Find Blob
        contours, hierarchy = cv2.findContours(th_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        ret_pos = (-1, -1, -1)

        area_count = 0

        copter_count = 0

        for h, cnt in enumerate(contours):

            area = cv2.contourArea(cnt)

            # Copter area
            if area > 200 and area < 1800:

                copter_count += 1

                # moments = cv2.moments(cnt)
                # Draw bounding Box
                # Get bounding box
                x,y,w,h = cv2.boundingRect(cnt)

                copter_rect = raw_depth[y: y+h, x: x+w]

                max_freq_depth = 0
                hist, bin_edges = np.histogram(copter_rect, bins=np.arange(1023))
                for i in range(len(hist)):
                    if hist[i] <= 0: continue
                    if hist[i] > max_freq_depth:
                        max_freq_depth = bin_edges[i]
                    # logging.info("{} : {}".format(bin_edges[i], hist[i]))

                # self._mean_arr_x.append(x+(w/2))
                # self._mean_arr_y.append(y+(h/2))
                self._mean_arr_z.append(max_freq_depth)

                mean_x = x+(w/2)
                mean_y = y+(h/2) #int(np.mean(self._mean_arr_y))
                mean_z = np.mean(self._mean_arr_z)

                mean_z = int(self._raw_depth_to_cm(mean_z))

                ret_pos = (mean_x, mean_y, mean_z)
                # logging.info('{}'.format(ret_pos))

                # Remove data in array
                # if len(self._mean_arr_x) > 3:
                #     self._mean_arr_x.pop(0)

                # if len(self._mean_arr_y) > 3:
                #     self._mean_arr_y.pop(0)

                if len(self._mean_arr_z) > 6:
                    self._mean_arr_z.pop(0)


                # logging.info("median : {}  max freq : {}".format(depth_median_val, max_freq_depth))
                # self.raw_depth_arr.append(max_freq_depth)

                # real_depth = reduce(lambda x, y: x + y, self.raw_depth_arr) / len(self.raw_depth_arr)

                # k1 = 1.1863
                # k2 = 2842.5
                # k3 = 0.1236
                #
                # pos = [x+(w/2), y+(h/2)]
                # real_depth_in_cm = int(100 * (k3 * math.tan(real_depth / k2 + k1)))

                # draw rotate bounding box
                # rect = cv2.minAreaRect(cnt)
                # box = cv2.cv.BoxPoints(rect)
                # box = np.int0(box)
                # cv2.drawContours(color_img,[box],0,(0,0,255),2)
                if EMIT_IMAGE:
                    # Draw bounding rect
                    cv2.rectangle(cvRGBImg,(x,y), (x+w,y+h), Scalar(255,0,0), 1)

                    # Draw text on image
                    cv2.circle(cvRGBImg,(mean_x, mean_y), 2, (0,0,255), -1)
                    str_out = '{}, {}'.format(mean_x, mean_y)
                    cv2.putText(cvRGBImg, str_out, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
                    # cv2.putText(cvRGBImg, str_out, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)

            # mask = np.zeros(depth_image.shape,np.uint8)
            # cv2.drawContours(mask,[cnt],0,255,-1)
            # mean = cv2.mean(im,mask = mask)
            pass

        # print 'area found : {}'.format(copter_count)
        if EMIT_IMAGE:
            self.ImageUpdated.emit(cvRGBImg)

        # Depth Filter
        if ret_pos is not None and (ret_pos[2] > self.max_depth_in_cm or ret_pos[2] < self.min_depth_in_cm):
            # self.PositionUpdated.emit(0, 0, 0)
            return (is_getpos_success, pos_result)

        # logging.info("area count : {}".format(area_count))

        if copter_count == 1:
            # logging.info('{}'.format(ret_pos))

            # self.DepthUpdated.emit("depth : {} cm".format(ret_pos[2]))
            return (True, ret_pos)

        # self.PositionUpdated.emit(0, 0, 0)
        return (is_getpos_success, pos_result)

    def set_target_x(self, x):
        self.set_x = x

    def set_target_y(self, y):
        self.set_y = y
        self.np_pid.SetTargetPoint(self.set_y)


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

        while True:

            start_time = time.clock()
            # self.get_image()
            found, pos = self.get_position()

            if found:
                self.PositionUpdated.emit(pos[0], pos[1], pos[2], self.set_x, self.set_y, self.set_z)

                diff_time = (time.clock() - start_time) * 1000

                if self._is_logging:
                    self.file.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(diff_time, pos[0], pos[1], pos[2], self.set_x, self.set_y, self.set_z))

                self._copter_pos = pos

                # logging.info('pos : {}'.format(pos))

                # agg_r_tuning = True if math.fabs(self.set_x - pos[0]) > 10 else False

                # agg_p_tuning = True if math.fabs(self.set_y - pos[1]) > 15 else False
                #
                # if agg_p_tuning != self._last_agg_pitch:
                #     self._last_agg_pitch = agg_p_tuning
                #
                #     if agg_p_tuning:
                #         self.p_pid.tuning(0.1, 1.0, 0.00025)
                #     else:
                #         self.p_pid.tuning(0.075, 1.0, 0.00025)

                    # logging.info('agg tuning : {}'.format(agg_p_tuning))

                # agg_t_tuning = True if math.fabs(self.set_z - pos[2]) > 10 else False

                # if agg_r_tuning:
                #     self.r_pid.tuning(0.05, 1.0, 0.00025)
                # else:
                #     self.r_pid.tuning(0.025, 1.0, 0.00025)
                #
                # if agg_p_tuning:
                #
                # else:
                #     self.p_pid.tuning(0.025, 1.0, 0.00025)

                self.p_pid.tuning(0.1, 1.0, 0.00025)

                self.p_pid.tuning(0.035, 1.0, 0.00035)

                # self.r_pid.tuning(0.05, 1.0, 0.0003)


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
                    # logging.info("thrust = {}".format(thrust))
                else:
                    thrust = self._last_thrust

                if self.np_pid.Compute(pos[1]):
                    new_pitch_sp = self.np_pid.GetOutput()
                else:
                    new_pitch_sp = self._last_pitch

                roll_sp = -roll
                pitch_sp = pitch
                thrust_sp = (thrust) + 40000



                self._last_thrust = thrust
                self._last_roll = roll
                self._last_pitch = new_pitch_sp
                # self._last_pitch = pitch

                if roll_sp > self.LIMIT: roll_sp = self.LIMIT
                elif roll_sp < -self.LIMIT: roll_sp = -self.LIMIT

                if pitch_sp > self.LIMIT: pitch_sp = self.LIMIT
                elif pitch_sp < -self.LIMIT: pitch_sp = -self.LIMIT

                # if thrust_sp > 63000: thrust_sp = 63000
                # elif thrust_sp < 0: thrust_sp = 0

                self.calc_pitch = pitch_sp
                self.calc_roll = roll_sp
                self.thrust = thrust_sp

                self.OutputUpdated.emit(self.calc_roll, new_pitch_sp, self.thrust)

                # logging.info('th pad : {}, th : {}'.format(self.thrust_pad, thrust_sp))

            self.trim_pitch = 0
            self.trim_roll = 0

            self.final_roll = self.calc_roll + self.trim_roll
            self.final_pitch = self.calc_pitch + self.trim_pitch

            if self.fly_en:
                if found:
                    self._is_copter_found = True
                else:
                    self._is_copter_found = False

                self.cf.commander.send_setpoint(self.final_roll, self.final_pitch, 0, self.thrust)
            else:
                self.cf.commander.send_setpoint(0, 0, 0, self.thrust_pad)


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

    def get_image(self):

        raw_depth = freenect.sync_get_depth()[0]

        self.last_raw_depth = np.copy(raw_depth)

        depth_image = frame_convert.my_depth_convert(raw_depth, self.max_raw_depth, self.min_raw_depth)


        # raw_rgb = freenect.sync_get_video()[0]

        cvRGBImg = cv2.cvtColor(depth_image, cv2.cv.CV_GRAY2RGB)



        self.ImageUpdated.emit(cvRGBImg)