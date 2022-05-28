import matplotlib.pyplot as plt
import numpy as np
from math import  radians ,copysign, sqrt, pow, pi, atan2, asin


class PID:
    def __init__(self):
        self.z_kp = 250
        self.z_ki = 4
        self.z_kd = 1
        
        self.x_kp = 30
        self.x_ki = 3
        self.x_kd = 2

        self.theta_kp = -60
        self.theta_ki = -10
        self.theta_kd = -10

        self.dt = 1/10


        self.x_error = 0
        self.pre_x_error = 0
        self.x_error_sum = 0

        self.z_error = 0
        self.pre_z_error = 0
        self.z_error_sum = 0


        self.theta_error = 0
        self.pre_theta_error = 0
        self.theta_error_sum = 0


    def x_control(self, cur_distance, target_distance):
        self.pre_x_error_ = self.x_error
        self.x_error = cur_distance - target_distance
        self.x_error_sum += self.x_error

        self.x_PID_control = self.x_kp * self.x_error + self.x_ki * self.x_error_sum * self.dt + self.x_kd * (self.x_error - self.pre_x_error_) / self.dt
      
        return self.x_PID_control


    def z_control(self, cur_distance, target_distance):
        self.pre_z_error_ = self.z_error
        self.z_error = cur_distance - target_distance
        self.z_error_sum += self.z_error

        self.z_PID_control = self.z_kp * self.z_error + self.z_ki * self.z_error_sum * self.dt + self.z_kd * (self.z_error - self.pre_z_error_) / self.dt
        return self.z_PID_control

    def theta_control(self, cur_distance, target_distance):
        self.pre_D_error_ = self.theta_error
        self.theta_error = cur_distance - target_distance
        self.theta_error_sum += self.theta_error

        self.theta_PID_control = self.theta_kp * self.theta_error + self.theta_ki * self.theta_error_sum * self.dt + self.theta_kd * (self.theta_error - self.pre_theta_error) / self.dt
        return self.theta_PID_control


    def gain_string(self):
        self.K = [self.z_kp, self.z_ki, self.z_kd]
        return  str(self.K)
