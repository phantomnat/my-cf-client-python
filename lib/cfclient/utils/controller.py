__author__ = 'messierz'

import time
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
from pid import PID_RP, PID

class Controller(QtCore.QThread):

    PlotUpdated = Caller()
    # PIDDataUpdated = QtCore.pyqtSignal(object, float)
    PositionUpdated = QtCore.pyqtSignal(int, int, int)

    ImageUpdated = QtCore.pyqtSignal(QPixmap);

    ThrustUpdated = Caller()


    last_raw_depth = None
    raw_depth_arr = []
    max_raw_depth = 925
    min_raw_depth = 750

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

        k1 = 1.1863
        k2 = 2842.5
        k3 = 0.1236

        # Max depth : 208 cm, Min depth : 101 cm
        self.min_depth_in_cm = int(100 * (k3 * math.tan(self.min_raw_depth / k2 + k1)))
        self.max_depth_in_cm = int(100 * (k3 * math.tan(self.max_raw_depth / k2 + k1)))

        self.r_pid = PID_RP(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        # self.r_pid = PID_RP(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        self.p_pid = PID_RP(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        # self.p_pid = PID_RP(P=0.05, D=1.0, I=0.00025, set_point=0.0)
        self.t_pid = PID(P=40, D=400.0, I=40, set_point=0.0)

        self.set_x = 320
        self.set_y = 240
        self.set_z = 130

        self.calc_pitch = 0
        self.calc_roll = 0

        self.LIMIT = 20

        self.trim_roll = 0
        self.trim_pitch = 0

    def set_auto_fly(self, en):
        logging.info('auto fly : {}'.format(en))

        self.fly_en = en

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


    def get_position(self):
        global last_depth_data


        raw_depth = freenect.sync_get_depth()[0]

        self.last_raw_depth = np.copy(raw_depth)

        depth_image = frame_convert.my_depth_convert(raw_depth, self.max_raw_depth, self.min_raw_depth)


        ret, th_img = cv2.threshold(depth_image, 240, 255, cv2.THRESH_BINARY )

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

        # color_img = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)

        # print len(contours)

        # for
        # cv2.drawContours(color_img,contours,-1,(0,255,0),5)
        ret_pos = (-1, -1, -1)
        area_count = 0

        for h,cnt in enumerate(contours):

            area = cv2.contourArea(cnt)

            # Check area
            if area > 100 and area < 5000:

                area_count += 1

                # moments = cv2.moments(cnt)
                # Draw bounding Box
                x,y,w,h = cv2.boundingRect(cnt)
                # cv2.rectangle(color_img,(x,y),(x+w,y+h),(0,255,0),2)

                last_raw_depth_crop = self.last_raw_depth[y: y+h, x: x+w]
                depth_median_val = np.ma.extras.median(last_raw_depth_crop)

                self.raw_depth_arr.append(depth_median_val)

                real_depth = reduce(lambda x, y: x + y, self.raw_depth_arr) / len(self.raw_depth_arr)

                k1 = 1.1863
                k2 = 2842.5
                k3 = 0.1236

                pos = [x+(w/2), y+(h/2)]
                real_depth_in_cm = int(100 * (k3 * math.tan(real_depth / k2 + k1)))

                ret_pos = (pos[0], pos[1], real_depth_in_cm)

                # Remove store data
                if len(self.raw_depth_arr) > 10:
                    self.raw_depth_arr.pop(0)

                # draw rotate bounding box
                # rect = cv2.minAreaRect(cnt)
                # box = cv2.cv.BoxPoints(rect)
                # box = np.int0(box)
                # cv2.drawContours(color_img,[box],0,(0,0,255),2)

                # Draw text on image
                # font = cv2.FONT_HERSHEY_SIMPLEX
                # cv2.circle(color_img,(pos[0],pos[1]), 2, (0,0,255), -1)
                # str_out = 'x : {} , y : {} , depth : {}'.format(pos[0],pos[1],real_depth)
                # cv2.putText(color_img, str_out, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
                # cv2.putText(color_img, str_out, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)

            # mask = np.zeros(depth_image.shape,np.uint8)
            # cv2.drawContours(mask,[cnt],0,255,-1)
            # mean = cv2.mean(im,mask = mask)
            pass

        # print 'area found : {}'.format(area_found)

        # Depth Filter
        if ret_pos is not None and (ret_pos[2] > self.max_depth_in_cm or ret_pos[2] < self.min_depth_in_cm):
            # self.PositionUpdated.emit(0, 0, 0)
            return (False, (-1, -1, -1))

        if area_count == 1:
            self.PositionUpdated.emit(ret_pos[0], ret_pos[1], ret_pos[2])
            return (True, ret_pos)

        # self.PositionUpdated.emit(0, 0, 0)
        return (False, (-1, -1, -1))

    def set_target_x(self, x):
        self.set_x = x

    def set_target_y(self, y):
        self.set_y = y

    def set_target_z(self, z):
        self.set_z = z

    def get_image(self):
        raw_depth = freenect.sync_get_depth()[0]

        self.last_raw_depth = np.copy(raw_depth)

        depth_image = frame_convert.my_depth_convert(raw_depth, self.max_raw_depth, self.min_raw_depth)


        # raw_rgb = freenect.sync_get_video()[0]

        cvRGBImg = cv2.cvtColor(depth_image, cv2.cv.CV_GRAY2RGB)

        qimg = QtGui.QImage(cvRGBImg.data,cvRGBImg.shape[1], cvRGBImg.shape[0], QtGui.QImage.Format_RGB888)
        # qimg = QtGui.QImage(raw_rgb.data,raw_rgb.shape[1], raw_rgb.shape[0], QtGui.QImage.Format_RGB888)

        # image = QImage(raw_rgb.tostring(), raw_rgb.width, raw_rgb.height, QImage.Format_RGB888).rgbSwapped()
        # pixmap = QPixmap.fromImage(image)
        pixmap = QtGui.QPixmap.fromImage(qimg)

        self.ImageUpdated.emit(pixmap)

    def copter_found(self):
        return self._is_copter_found

    def get_copter_position(self):
        if self._is_copter_found:
            return ()
            pass

    def run(self):
        d_time_count = 0
        n_frame_count = 0

        while True:
            start_time = time.clock()

            # self.get_image()

            found, pos = self.get_position()

            self._is_copter_found = found

            if found:

                self._copter_pos = pos

                # logging.info('pos : {}'.format(pos))
                agg_r_tuning = True if math.fabs(self.set_x - pos[0]) > 10 else False

                agg_p_tuning = True if math.fabs(self.set_y - pos[1]) > 10 else False

                agg_t_tuning = True if math.fabs(self.set_z - pos[2]) > 10 else False

                if agg_r_tuning:
                    self.r_pid.tuning(0.05, 1.0, 0.00025)
                else:
                    self.r_pid.tuning(0.025, 1.0, 0.00025)

                if agg_p_tuning:
                    self.p_pid.tuning(0.05, 1.0, 0.00025)
                else:
                    self.p_pid.tuning(0.025, 1.0, 0.00025)

                if agg_t_tuning:
                    self.t_pid.tuning(60, 500, 40)
                else:
                    self.t_pid.tuning(20, 500, 40)


                roll = self.r_pid.update(self.set_x - pos[0])
                pitch = self.p_pid.update(self.set_y - pos[1])
                thrust = self.t_pid.update(self.set_z - pos[2])

                roll_sp = -roll
                pitch_sp = pitch
                thrust_sp = (thrust) + 42000

                if roll_sp > self.LIMIT: roll_sp = self.LIMIT
                elif roll_sp < -self.LIMIT: roll_sp = -self.LIMIT

                if pitch_sp > self.LIMIT: pitch_sp = self.LIMIT
                elif pitch_sp < -self.LIMIT: pitch_sp = -self.LIMIT

                if thrust_sp > 63000: thrust_sp = 63000
                elif thrust_sp < 0: thrust_sp = 0

                self.calc_pitch = pitch_sp
                self.calc_roll = roll_sp
                self.thrust = thrust_sp

                # logging.info('th pad : {}, th : {}'.format(self.thrust_pad, thrust_sp))

            self.trim_pitch = 0
            self.trim_roll = 0

            self.final_roll = self.calc_roll + self.trim_roll
            self.final_pitch = self.calc_pitch + self.trim_pitch

            if self.fly_en:
                self.cf.commander.send_setpoint(self.final_roll, self.final_pitch, 0, self.thrust)
            else:
                self.cf.commander.send_setpoint(0, 0, 0, self.thrust_pad)



            end_time = time.clock()

            d_time_count += ((end_time - start_time) * 1000)
            n_frame_count += 1

            if n_frame_count > 10:
                # logging.info('avg time : {}'.format(d_time_count / n_frame_count))
                # logging.info("max depth : {} cm,  min depth : {}".format(self.max_depth_in_cm, self.min_depth_in_cm))

                d_time_count = 0
                n_frame_count = 0

            time.sleep(0.0001)
