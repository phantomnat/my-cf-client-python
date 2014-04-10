#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __                           
#  +------+      / __ )(_) /_______________ _____  ___ 
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2013 Bitcraze AB
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
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

import math
import time

class PID:

    def __init__(self, P=1.0, I=0.0, D=10.0, Derivator=0, Integrator=0,
                 Integrator_max=300, Integrator_min=-200, set_point=0.0,
                 power=1.0):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.power = power
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min
        self.last_error = 0.0
        self.last_value = 0.0

        self.set_point=set_point
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        if (self.last_value >= current_value):
            change = self.error - self.last_error
        else:
            change = 0.0

        if self.error > 0.0:
            self.I_value = self.Integrator * self.Ki
        else:
            self.I_value = (self.Integrator * self.Ki)


        #self.D_value = self.Kd * ( self.error - self.Derivator)
        self.D_value = self.Kd * change
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error/200.0

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.last_error = self.error
        self.last_value = current_value

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def set_point(self,set_point):
        """Initilize the setpoint of PID"""
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

class PID_RP:

    def __init__(self, P=1.0, I=0.0, D=10.0, Derivator=0, Integrator=0,
                 Integrator_max=20000, Integrator_min=-20000, set_point=0.0,
                 power=1.0):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.power = power
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min
        self.last_error = 0.0
        self.last_value = 0.0

        self.set_point=set_point
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        change = self.error - self.last_error
        
        self.I_value = self.Integrator * self.Ki

        #self.D_value = self.Kd * ( self.error - self.Derivator)
        self.D_value = self.Kd * change
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.last_error = self.error
        self.last_value = current_value

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def set_point(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def reset(self):
        self.Integrator=0
        self.Derivator=0

    def tuning(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

class ArduinoPID:

    myInput = 0
    myOutput = 0
    mySetpoint = 0

    inAuto = False
    kp = 0
    ki = 0
    kd = 0
    controllerDirection = 0
    dispKp = 0
    dispKi = 0
    dispKd = 0

    ITerm = 0
    lastInput = 0
    outMin = 0.0
    outMax = 0.0
    lastTime = 0.0

    def __init__(self, Input, Output, Setpoint, Kp, Ki, Kd, ControllerDirection = 0):
        self.myInput = Input
        self.myOutput = Output
        self.mySetpoint = Setpoint
        self.inAuto = False

        self.SetOutputLimits(0, 100)

        self.SampleTime = 20    # 20 ms

        self.SetControllerDirection(ControllerDirection)
        self.SetTuning(Kp, Ki, Kd)

        self.lastTime = (time.clock() * 1000) - self.SampleTime

    def GetOutput(self):
        return self.myOutput

    def Compute(self, input):
        if not self.inAuto: return False
        now = time.clock() * 1000
        timeChange = now - self.lastTime
        self.myInput = input

        if timeChange > self.SampleTime:

            error = self.mySetpoint - input
            self.ITerm += (self.ki * error)

            if self.ITerm > self.outMax: self.ITerm = self.outMax
            elif self.ITerm < self.outMin: self.ITerm = self.outMin

            deltaInput = input - self.lastInput

            output = self.kp * error + self.ITerm - self.kd * deltaInput

            if output > self.outMax: output = self.outMax
            elif output < self.outMin: output = self.outMin

            self.myOutput = output
            self.lastInput = input
            self.lastTime = now

            return True

        return False

    def SetTuning(self, kp, ki, kd):
        if kp < 0 or ki < 0 or kd < 0: return

        self.dispKp = kp
        self.dispKi = ki
        self.dispKd = kd

        SampleTimeInSec = self.SampleTime / 1000.0

        self.kp = kp
        self.ki = ki * SampleTimeInSec
        self.kd = kd / SampleTimeInSec

        if self.controllerDirection == 1:
            self.kp = 1 - self.kp
            self.ki = 1 - self.ki
            self.kd = 1 - self.kd

    def SetSampleTime(self, NewSampleTime):
        if NewSampleTime > 0:
            ratio = NewSampleTime / float(self.SampleTime)
            self.ki += ratio
            self.kd /= ratio
            self.SampleTime = NewSampleTime

    def SetOutputLimits(self, _min, _max):
        if min > max: return
        self.outMax = _max
        self.outMin = _min

        if self.inAuto:
            if self.myOutput > self.outMax: self.myOutput = self.outMax
            elif self.myOutput < self.outMin: self.myOutput = self.outMin

            if self.ITerm > self.outMax: self.ITerm = self.outMax
            elif self.ITerm < self.outMin: self.ITerm = self.outMin

    def SetMode(self, mode = True):
        if mode == ~self.inAuto:
            self.Initialize()
        self.inAuto = mode

    def Initialize(self):
        self.ITerm = self.myOutput
        self.lastInput= self.myInput
        if self.ITerm > self.outMax:
            self.ITerm = self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm = self.outMin

    def SetControllerDirection(self, direction):
        if self.inAuto and direction != self.controllerDirection:
            self.kp = 1 - self.kp
            self.ki = 1 - self.ki
            self.kd = 1 - self.kd
        self.controllerDirection = direction