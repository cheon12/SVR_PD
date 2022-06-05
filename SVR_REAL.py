import matplotlib.pyplot as plt
import numpy as np
from math import  radians ,copysign, sqrt, pow, pi, atan2, asin
from sklearn.svm import SVR


class SVR1:
    def __init__(self):
        self.z_kp = 15
        self.z_ki = 0
        self.z_kd = 10
        
        self.x_kp = 5
        self.x_ki = 0
        self.x_kd = 6

        self.theta_kp = 10
        self.theta_ki = 0
        self.theta_kd = 15


        self.dt = 1/10


        self.x_error = 0
        self.x1_error=0
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
        self.x_error = 1000*(cur_distance - target_distance)+self.theta_error
        self.x1_error = (cur_distance - target_distance)
        self.x_error_sum += self.x_error
        #self.x_PID_control = 0.1*(130.986*self.x_error+35.76) + self.x_ki*self.x_error_sum*self.dt+ self.x_kd * (self.x_error - self.pre_x_error_) / self.dt
        self.x_PID_control = self.x_kp * self.x_error + 0*(65.99*self.x_error_sum+15.988)+ self.x_kd * (self.x_error - self.pre_x_error_) / self.dt
        self.x1=[]
        self.x2=[]
        self.x1.append(self.x_error_sum)
        self.x2.append(self.x_PID_control)
        self.noise = np.random.normal(0,1,40)
        self.x2+=self.noise*2
        return self.x_PID_control


    def z_control(self, cur_distance, target_distance):
        self.pre_z_error_ = self.z_error
        self.z_error = 50*(-cur_distance + target_distance)+self.x1_error
        self.z_error_sum += self.z_error
        #self.z_PID_control = 0.1*(230.83*self.z_kp-37.14) + self.z_ki*self.z_error_sum *self.dt + self.z_kd * (self.z_error - self.pre_z_error_) / self.dt
        self.z_PID_control = self.z_kp * self.z_error + 0*(115.91 * self.z_error_sum -18.74) + self.z_kd * (self.z_error - self.pre_z_error_) / self.dt
        self.z1=[]
        self.z2=[]
        self.z1.append(self.z_error_sum)
        self.z2.append(self.z_PID_control)
        self.noise = np.random.normal(0,1,40)
        self.z2+=self.noise*2
        return self.z_PID_control


    def theta_control(self, cur_distance, target_distance):
        self.pre_theta_error_ = self.theta_error
        self.theta_error = cur_distance - target_distance
        self.theta_error_sum += self.theta_error
        #self.theta_PID_control = 0.1*(313.12*self.theta_kp+39.86) + self.theta_ki*self.theta_error_sum*self.dt+ self.theta_kd * (self.theta_error - self.pre_theta_error) / self.dt
        self.theta_PID_control = self.theta_kp * self.theta_error +0*(+156.98 * self.theta_error_sum +18.64) + self.theta_kd * (self.theta_error - self.pre_theta_error) / self.dt
        self.theta1=[]
        self.theta2=[]
        self.theta1.append(self.theta_error_sum)
        self.theta2.append(self.theta_PID_control)
        self.noise = np.random.normal(0,1,40)
        self.theta2+=self.noise*2
        return self.theta_PID_control



    def gain_string(self):
        self.K = [self.z_kp, self.z_ki, self.z_kd]
        return  str(self.K)


a=list()
b=list()
k=SVR1()
random=np.random.rand(40)
k.x_control(random*10,0)
k.z_control(random*10,0)
k.theta_control(random*10,0)

plt.subplot(3,1,1)
plt.plot(k.x1,k.x2,'ro',markersize=3)
plt.subplot(3,1,2)
plt.plot(k.z1,k.z2,'go',markersize=3)
plt.subplot(3,1,3)
plt.plot(k.theta1,k.theta2,'bo',markersize=3)
plt.show()

k.x1=np.array(k.x1)
k.x2=np.array(k.x2)
k.z1=np.array(k.z1)
k.z2=np.array(k.z2)
k.theta1=np.array(k.theta1)
k.theta2=np.array(k.theta2)
k.x1=k.x1.reshape(-1,1)
k.x2=k.x2.reshape(-1)
k.z1=k.z1.reshape(-1,1)
k.z2=k.z2.reshape(-1)
k.theta1=k.theta1.reshape(-1,1)
k.theta2=k.theta2.reshape(-1)



model1=SVR(kernel='linear',C=1000,epsilon=20)
model2=SVR(kernel='linear',C=1000,epsilon=20)
model3=SVR(kernel='linear',C=1000,epsilon=20)

model1.fit(k.x1,k.x2)
model2.fit(k.z1,k.z2)
model3.fit(k.theta1,k.theta2)

relation_square_x=model1.score(k.x1,k.x2)
relation_square_z=model2.score(k.z1,k.z2)
relation_square_theta=model3.score(k.theta1,k.theta2)

print('결정계수 x_R: ',relation_square_x)
print('결정계수 z_R: ',relation_square_z)
print('결정계수 theta_R: ',relation_square_theta)

y_px=model1.predict(k.x1)
y_pz=model2.predict(k.z1)
y_pt=model3.predict(k.theta1)

print("x_coef   ",model1.coef_)
print("x_intercept    ",model1.intercept_)
print("z_coef   ",model2.coef_)
print("z_intercept    ",model2.intercept_)
print("theta_coef   ",model3.coef_)
print("theta_intercept    ",model3.intercept_)
plt.subplot(3,1,1)
plt.scatter(k.x1,k.x2,marker='+')
plt.scatter(k.x1,y_px,marker='o')

plt.subplot(3,1,2)
plt.scatter(k.z1,k.z2,marker='+')
plt.scatter(k.z1,y_pz,marker='o')

plt.subplot(3,1,3)
plt.scatter(k.theta1,k.theta2,marker='+')
plt.scatter(k.theta1,y_pt,marker='o')
plt.show()





















# b=[list(k.x1[0])]
# b1=np.array(b)
# k.x1=np.array(b1)
# k.x1=np.transpose(k.x1)
# for i in range(len(k.x2[0])):
#     a.append(k.x2[0][i])
# k.x2=np.array(a)

# print(k.x1)
# print(k.x2)



# k.x1=k.x1.reshape(-1,1)
# k.x2=k.x2.reshape(-1,1)


# #print(k.x1.shape)
# #print(k.x2.shape)
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
#         k.x1,
#         svr.fit(k.x1, k.x2).predict(k.x1),
#         color=model_color[ix],
#         lw=lw,
#         label="{} model".format(kernel_label[ix]),
#     )
#     axes[ix].scatter(
#         k.x1[svr.support_],
#         k.x2[svr.support_],
#         facecolor="none",
#         edgecolor=model_color[ix],
#         s=50,
#         label="{} support vectors".format(kernel_label[ix]),
#     )
#     axes[ix].scatter(
#         k.x1[np.setdiff1d(np.arange(len(k.x1)), svr.support_)],
#         k.x2[np.setdiff1d(np.arange(len(k.x1)), svr.support_)],
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