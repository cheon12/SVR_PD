import matplotlib.pyplot as plt
import numpy as np
from math import  radians ,copysign, sqrt, pow, pi, atan2, asin
from sklearn.svm import SVR


class SVR1:
    def __init__(self):
        self.z_kp = 250
        self.z_ki = 400
        self.z_kd = 1
        
        self.x_kp = 30
        self.x_ki = 300
        self.x_kd = 2

        self.theta_kp = -60
        self.theta_ki = -100
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

        self.self.model1=self.model1
        self.self.model2=self.model2
        self.self.model3=self.model3


    def x_control(self, cur_distance, target_distance):
        self.pre_x_error_ = self.x_error
        self.x_error = cur_distance - target_distance
        self.x_error_sum += self.x_error



        self.x_PID_control = self.x_kp * self.x_error + self.x_error_sum*self.self.model1.coef_+self.self.model1.intercept_ + self.x_kd * (self.x_error - self.pre_x_error_) / self.dt
        self.x1=[]
        self.x2=[]
        self.x1.append(self.x_error_sum)
        self.x2.append(self.x_PID_control)
        self.noise = np.random.normal(0,1,40)
        self.x2+=self.noise*0.2
        return self.x_PID_control


    def z_control(self, cur_distance, target_distance):
        self.pre_z_error_ = self.z_error
        self.z_error = cur_distance - target_distance
        self.z_error_sum += self.z_error

        self.z_PID_control = self.z_kp * self.z_error + self.z_error_sum*self.model2.coef_+self.model2.intercept_ + self.z_kd * (self.z_error - self.pre_z_error_) / self.dt
        self.z1=[]
        self.z2=[]
        self.z1.append(self.z_error_sum)
        self.z2.append(self.z_PID_control)
        self.noise = np.random.normal(0,1,40)
        self.z2+=self.noise*0.2
        return self.z_PID_control


    def theta_control(self, cur_distance, target_distance):
        self.pre_D_error_ = self.theta_error
        self.theta_error = cur_distance - target_distance
        self.theta_error_sum += self.theta_error

        self.theta_PID_control = self.theta_kp * self.theta_error + self.theta_error_sum*self.model3.coef_+self.model3.intercept_ + self.theta_kd * (self.theta_error - self.pre_theta_error) / self.dt
        self.theta1=[]
        self.theta2=[]
        self.theta1.append(self.theta_error_sum)
        self.theta2.append(self.theta_PID_control)
        self.noise = np.random.normal(0,1,40)
        self.theta2+=self.noise*0.2
        return self.theta_PID_control



    def gain_string(self):
        self.K = [self.z_kp, self.z_ki, self.z_kd]
        return  str(self.K)

    def svrr(self):
        self.a=list()
        self.b=list()
        random=np.random.rand(40)
        self.x_control(random*10,0)
        self.z_control(random*10,0)
        self.theta_control(random*10,0)
        self.x1=np.array(self.x1)
        self.x2=np.array(self.x2)
        self.z1=np.array(self.z1)
        self.z2=np.array(self.z2)
        self.theta1=np.array(self.theta1)
        self.theta2=np.array(self.theta2)
        self.x1=self.x1.reshape(-1,1)
        self.x2=self.x2.reshape(-1)
        self.z1=self.z1.reshape(-1,1)
        self.z2=self.z2.reshape(-1)
        self.theta1=self.theta1.reshape(-1,1)
        self.theta2=self.theta2.reshape(-1)



        self.model1=SVR(kernel='linear',C=1000,epsilon=20)
        self.model2=SVR(kernel='linear',C=1000,epsilon=20)
        self.model3=SVR(kernel='linear',C=1000,epsilon=20)

        self.model1.fit(self.x1,self.x2)
        self.model2.fit(self.z1,self.z2)
        self.model3.fit(self.theta1,self.theta2)

        relation_square_x=self.model1.score(self.x1,self.x2)
        relation_square_z=self.model2.score(self.z1,self.z2)
        relation_square_theta=self.model3.score(self.theta1,self.theta2)

        print('결정계수 x_R: ',relation_square_x)
        print('결정계수 z_R: ',relation_square_z)
        print('결정계수 theta_R: ',relation_square_theta)

        self.y_px=self.model1.predict(self.x1)
        self.y_pz=self.model2.predict(self.z1)
        self.y_pt=self.model3.predict(self.theta1)

        print("x_coef   ",self.model1.coef_)
        print("x_intercept    ",self.model1.intercept_)
        print("z_coef   ",self.model2.coef_)
        print("z_intercept    ",self.model2.intercept_)
        print("theta_coef   ",self.model3.coef_)
        print("theta_intercept    ",self.model3.intercept_)
        plt.subplot(3,1,1)
        plt.scatter(self.x1,self.x2,marker='+')
        plt.scatter(self.x1,self.y_px,marker='o')

        plt.subplot(3,1,2)
        plt.scatter(self.z1,self.z2,marker='+')
        plt.scatter(self.z1,self.y_pz,marker='o')

        plt.subplot(3,1,3)
        plt.scatter(self.theta1,self.theta2,marker='+')
        plt.scatter(self.theta1,self.y_pt,marker='o')
        plt.show()





















# b=[list(self.x1[0])]
# b1=np.array(b)
# self.x1=np.array(b1)
# self.x1=np.transpose(self.x1)
# for i in range(len(self.x2[0])):
#     a.append(self.x2[0][i])
# self.x2=np.array(a)

# print(self.x1)
# print(self.x2)



# self.x1=self.x1.reshape(-1,1)
# self.x2=self.x2.reshape(-1,1)


# #print(self.x1.shape)
# #print(self.x2.shape)
# svr_rbf = SVR(kernel="rbf", C=100, gamma=0.1, epsilon=0.1)
# svr_lin = SVR(kernel="linear", C=100, gamma="auto")
# svr_poly = SVR(kernel="poly", C=100, gamma="auto", degree=3, epsilon=0.1, coef0=1)

# lw = 2

# svrs = [svr_rbf, svr_lin, svr_poly]
# kernel_label = ["RBF", "Linear", "Polynomial"]
# model_color = ["m", "c", "g"]

# fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 10), sharey=True)
# for ix, svr in enumerate(svrs): #튜플 형태로 만들어줌
#     axes[ix].plot(
#         self.x1,
#         svr.fit(self.x1, self.x2).predict(self.x1),
#         color=model_color[ix],
#         lw=lw,
#         label="{} model".format(kernel_label[ix]),
#     )
#     axes[ix].scatter(
#         self.x1[svr.support_],
#         self.x2[svr.support_],
#         facecolor="none",
#         edgecolor=model_color[ix],
#         s=50,
#         label="{} support vectors".format(kernel_label[ix]),
#     )
#     axes[ix].scatter(
#         self.x1[np.setdiff1d(np.arange(len(self.x1)), svr.support_)],
#         self.x2[np.setdiff1d(np.arange(len(self.x1)), svr.support_)],
#         facecolor="none",
#         edgecolor="k",
#         s=50,
#         label="other training data",
#     )
#     axes[ix].legend(
#         loc="upper center",
#         bbox_to_anchor=(0.5, 1.1),
#         ncol=1,
#         fancybox=True,
#         shadow=True,
#     )

# fig.text(0.5, 0.04, "data", ha="center", va="center")
# fig.text(0.06, 0.5, "target", ha="center", va="center", rotation="vertical")
# fig.suptitle("Support Vector Regression", fontsize=14)
# plt.show()