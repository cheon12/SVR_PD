import matplotlib.pyplot as plt
import numpy as np
from math import  radians ,copysign, sqrt, pow, pi, atan2, asin

class SVR:
    def __init__(self):
        self.D_kp = 9
        self.D_ki = 9
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
        print(self.PID_control)
        self.x=[]
        self.y=[]
        self.x.append(self.D_error_sum)
        self.y.append(self.PID_control)
        self.noise = np.random.normal(0,1,100)
        self.y+=self.noise*10




        return self.PID_control





    def gain_string(self):
        self.K = [self.D_kp, self.D_ki, self.D_kd]
        return  str(self.K)


k=SVR()
random=np.random.rand(100)
k.Vcontrol(random*10,0)


plt.plot(k.x,k.y,'ro')
plt.show()