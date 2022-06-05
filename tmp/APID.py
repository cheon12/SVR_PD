import matplotlib.pyplot as plt 
import math

class APID:
    def __init__(self):
        #초기 계수 설정
        self.Kp = 9
        self.Ki = 1
        self.Kd = 1

        #학습률 설정
        self.learning_rate = 0.8
        
        #초기 계수의 변화량 변수
        self.dK1 = 0
        self.dK2 = 0
        self.dK3 = 0
        
        self.delta_dK1 = 0
        self.delta_dK2 = 0
        self.delta_dK3 = 0 
        
        self.zcount = 0

        #error항, y항, u항 (error=r-y, r=입력(이전 차간 거리), y=출력(현재 차간 거리), u=제어량)
        self.mat = [[0, 0, 0],  #error[(k-2), (k-1), k]
                    [0, 0, 0],  #y same as above
                    [0, 0, 0]]  #u same as above

        #제어량
        self.control_amount = 0
        
        self.common = 0
    
    def Vcontrol(self, cdistance, ref_distance):
        for i in range(0,3):
            for j in range(0,2):
                self.mat[i][j] = self.mat[i][j+1]
    
        self.mat[0][2] = cdistance - ref_distance
        self.mat[1][2] = cdistance
        self.mat[2][2] = self.control_amount

        # for i in range(0,3):
        #     for j in range(0,3):
        #         print("[",i,"][",j,"]",self.mat[i][j])
        # print("\n")

        # print("zcount = ",self.zcount)

        try:
            common_term = self.learning_rate * self.mat[0][2] * (self.mat[1][2] - self.mat[1][1]) / (self.mat[2][2] - self.mat[2][1])     
            self.delta_dK1 = common_term * (self.mat[0][2]-self.mat[0][1])
            self.delta_dK2 = common_term * self.mat[0][2]
            self.delta_dK3 = common_term * (self.mat[0][2]-2*self.mat[0][1]+self.mat[0][0])
        
       

        except ZeroDivisionError:
            print("ZeroDivision")
            self.zcount += 1
            if self.zcount == 1:
                self.mat[0][1]=self.mat[0][2]
            if self.zcount == 2:
                self.mat[0][0] = self.mat[0][1]
                self.mat[0][1] = self.mat[0][2]

        self.dK1 = self.dK1 + self.delta_dK1
        self.dK2 = self.dK2 + self.delta_dK2
        self.dK3 = self.dK3 + self.delta_dK3

        self.control_amount = self.control_amount + (self.Kp + self.dK1)*(self.mat[0][2]-self.mat[0][1]) \
            + (self.Ki + self.dK2)*self.mat[0][2] + (self.Kd + self.dK3)*(self.mat[0][2]-2*self.mat[0][1]+self.mat[0][0])

        return self.control_amount

    def gain_string(self):
        self.K = [self.Kp, self.Ki, self.Kd]
        return  str(self.K)