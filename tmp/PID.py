import matplotlib.pyplot as plt
import numpy as np
from math import  radians ,copysign, sqrt, pow, pi, atan2, asin


# dK1(k+1) = (-n) * (-e(k)) * {y(k-1)-y(k-2)}/{u(k-1)-u(k-2)} * (e(k)-e(k-1))
# dK2(k+1) = (-n) * (-e(k)) * {y(k-1)-y(k-2)}/{u(k-1)-u(k-2)} * e(k)
# dK3(k+1) = (-n) * (-e(k)) * {y(k-1)-y(k-2)}/{u(k-1)-u(k-2)} * {e(k)-2e(k-1)+e(k-2)}
# 위의 결과를 제어량 u(t)에 대입한다.

# u(t) = (D_kp + dK1)e(t) + (D_ki+dK2) * (적분항) + (D_kd+dK3) * (미분항)

class PID:
    def __init__(self):
        self.D_kp = 9
        self.D_ki = 1
        self.D_kd = 1


        self.D_error = 0
        self.dt = 1/10
        self.pre_D_error = 0
        self.D_error_sum = 0

    def Vcontrol(self, cur_distance, target_distance):
        self.pre_D_error_ = self.D_error
        self.D_error = cur_distance - target_distance
        self.D_error_sum += self.D_error

        self.PID_control = self.D_kp * self.D_error + self.D_ki * self.D_error_sum * self.dt + self.D_kd * (self.D_error - self.pre_D_error_) / self.dt
        return self.PID_control


    def gain_string(self):
        self.K = [self.D_kp, self.D_ki, self.D_kd]
        return  str(self.K)
