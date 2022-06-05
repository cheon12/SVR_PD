from PID_REAL import PID
import matplotlib.pyplot as plt
from time import sleep, time
import math
import sys
import csv    
import numpy as np
trigger = 0
class Control(PID):
    def __init__(self):
        super().__init__()
      
        self.PID = PID()
        self.data_z = {'goal_z':{'z' :0 , 'ddz' :0 }, 
                    'PID':{'z' : 20, 'ddz' : 0,'F_E':10, 'control': 0}
                  
                   }
        self.data_x= {'goal_x':{'x':0,'ddx':0},
                    'PID':{'x':0,'ddx':0,'F_S':10,'control':0}
        }    
        
        self.data_theta= {'goal_theta':{'theta':0,'ddtheta':0},
                    'PID':{'theta':1,'ddtheta':0,'pi':1,'control':0}
        }       



        self.dt = 1/100
        self.dt_list = []
        self.z_PID_list = []
        self.z_ref_list = []
        self.z1_ref_list = []
        
        self.z_ref = 0
        self.x_PID_list = []
        self.x_ref_list = []
        self.x_ref = 0

        self.theta_PID_list = []
        self.theta_ref_list = []
        self.theta_ref_list1 = []

        self.theta_ref = 0
        self.theta_ref1 = 3.14



    def V_cal(self):
        global trigger
        trigger += 1


        for key in self.data_x.keys():
            self.data_x['PID']['x'] += self.data_x['PID']['ddx'] *self.dt* self.dt/2



        for key in self.data_z.keys():
            self.data_z['PID']['z'] += self.data_z['PID']['ddz'] *self.dt* self.dt/2


        for key in self.data_theta.keys():
            self.data_theta['PID']['theta'] += self.data_theta['PID']['ddtheta'] *self.dt* self.dt/2


        V_control_z = {'PID': self.PID.z_control(self.data_z['PID']['z'] ,0)
        }
        V_control_x = {'PID': self.PID.x_control(self.data_x['PID']['x'] ,0)
        }
        V_control_theta = {'PID': self.PID.theta_control(self.data_theta['PID']['theta'] ,0)
        }        

        for key in self.data_x.keys():
            if key != 'goal_x':
                self.data_x['PID']['control']=V_control_x['PID']
                self.data_x['PID']['F_S']+=self.data_x['PID']['control']
                self.data_x['PID']['ddx']=((self.data_z['PID']['F_E']*math.cos(self.data_theta['PID']['pi'])*math.sin(self.data_theta['PID']['theta'])+self.data_z['PID']['F_E']*math.sin(self.data_theta['PID']['pi'])*math.cos(self.data_theta['PID']['theta'])+self.data_x['PID']['F_S']*math.cos(self.data_theta['PID']['theta']))/8.5)
                self.data_x['PID']['F_S'] = max(min(self.data_x['PID']['F_S'], 130), -130)


        for key in self.data_z.keys():
            if key != 'goal_z':
                self.data_z['PID']['control']=V_control_z['PID']
                self.data_z['PID']['F_E']+=self.data_z['PID']['control']
                self.data_z['PID']['ddz']=((self.data_z['PID']['F_E']*math.cos(self.data_theta['PID']['pi'])*math.cos(self.data_theta['PID']['theta'])-self.data_z['PID']['F_E']*math.sin(self.data_theta['PID']['pi'])*math.sin(self.data_theta['PID']['theta'])-self.data_x['PID']['F_S']*math.sin(self.data_theta['PID']['theta'])-83.3)/8.5)
                self.data_z['PID']['F_E'] = max(min(self.data_z['PID']['F_E'], 6486), 0)
                # if self.data_z['PID']['z']<5:
                #     self.data_z['PID']['z']=0        
            

        for key in self.data_theta.keys():
            if key != 'goal_theta':
                self.data_theta['PID']['control']=V_control_theta['PID']
                self.data_theta['PID']['pi']+=self.data_theta['PID']['control']
                self.data_theta['PID']['ddtheta']=((-self.data_z['PID']['F_E']*math.sin(self.data_theta['PID']['pi'])*(0.685+0.1*math.cos(self.data_theta['PID']['pi']))+(0.575*self.data_x['PID']['F_S']))/1.34)
                self.data_theta['PID']['pi'] = max(min(self.data_theta['PID']['pi'], 3.14), -3.14)

        self.dt_list.append(self.dt * trigger)

        self.z_PID_list.append(self.data_z['PID']['z'])
        self.x_PID_list.append(self.data_x['PID']['x'])
        self.theta_PID_list.append(self.data_theta['PID']['theta'])
        self.z_ref_list.append(self.z_ref)
        self.x_ref_list.append(self.x_ref) 
        self.theta_ref_list.append(self.theta_ref) 
        self.theta_ref_list1.append(self.theta_ref1) 

        self.PID_max = max(self.z_PID_list)
        self.PID_max = max(self.x_PID_list)
        self.PID_max = max(self.theta_PID_list)

        plt.subplot(4,1,1)
        plt.plot(self.dt_list, self.z_PID_list, 'b')
        plt.plot(self.dt_list, self.z_ref_list, 'g')
        plt.subplot(4,1,2)
        plt.plot(self.dt_list, self.x_PID_list, 'b')
        plt.plot(self.dt_list, self.x_ref_list, 'g')
        plt.subplot(4,1,3)
        plt.plot(self.dt_list, self.theta_PID_list, 'b')
        plt.plot(self.dt_list, self.theta_ref_list, 'g')
        plt.plot(self.dt_list, self.theta_ref_list1, 'g')


        plt.subplot(4,1,4)
        plt.plot(self.x_PID_list, self.z_PID_list, 'b')
        plt.plot(self.dt_list, self.theta_ref_list, 'g')
        plt.pause(0.001)
    
        print(f'V_control_z: {V_control_z}')
        print(f'V_control_x: {V_control_x}')
        print(f'V_control_theta: {V_control_theta}')
        print(f"z:{self.data_z['PID']['z']}")
        print(f"x:{self.data_x['PID']['x']}")
        print(f"theta:{self.data_theta['PID']['theta']}")
        print(f"F_E:{self.data_z['PID']['F_E']}")
        print(f"F_S:{self.data_x['PID']['F_S']}")
        print(f"pi:{self.data_theta['PID']['pi']}")
        print('=====================')
        print('self.data_z',self.data_z)
        print('self.data_x',self.data_x)
        print('self.data_theta',self.data_theta)

        return self.data_z['PID']['z']

C = Control()
file_name = C.gain_string()
f = open(f'csv/{file_name}.csv', 'w')
write_f = csv.writer(f)

while True:
    C.V_cal()
    if trigger == 1:
        write_f.writerow(['Time', 'PID_distance'])#,'APID_distance','SPID_distance'])        
    write_f.writerow([str(trigger), str(C.z_PID_list[trigger-1])])#, str(C.D_APID_list[trigger-1]),str(C.D_APID_list[trigger-1])])
    if trigger == 1600:
        write_f.writerow(['C.PID_max',str(C.PID_max)])      
        plt.savefig(f"image/{file_name}.png")
        sys.exit()
        f.close()
plt.show()

# plt.savefig('%s.png'%(B.gain_string()))