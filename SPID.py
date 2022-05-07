import matplotlib.pyplot as plt
import numpy as np
from math import  radians ,copysign, sqrt, pow, pi, atan2, asin




class SPID:
    def __init__(self):
        self.D_kp = 9
        #self.D_ki = 1
        self.D_kd = 1
        self.D_error = 0
        self.dt = 1/10
        self.pre_D_error = 0
        self.D_error_sum = 0


    #def SVR:


    def Vcontrol(self, cur_distance, target_distance):
        self.pre_D_error_ = self.D_error
        self.D_error = cur_distance - target_distance
        self.D_error_sum += self.D_error
        self.PID_control = self.D_kp * self.D_error + self.D_kd * (self.D_error - self.pre_D_error_) / self.dt
        return self.PID_control


    def gain_string(self):
        self.K = [self.D_kp, self.D_kd]
        return  str(self.K)
