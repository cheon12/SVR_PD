from PID_REAL import PID
#from APID import APID
#from SPID import SPID
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
        #클래스 지정
        # self.SPID=SPID()
        # self.APID = APID()
        self.PID = PID()
        self.data_z = {'goal_z':{'z' :0 , 'dz' :0 },  #D=1 V=0.3
                    'PID':{'z' : 0, 'dz' : 0,'F_E':30, 'control': 0}
                    # 'APID':{'D' : 0, 'V' : 0, 'control' : 0},
                    # 'SPID':{'D' : 0, 'V' : 0, 'control' : 0},
                   }
        self.data_x= {'goal_x':{'x':0,'dx':0},
                    'PID':{'x':0,'dx':0,'F_S':3,'control':0}
        }    
        
        self.data_theta= {'goal_theta':{'theta':0,'dtheta':0},
                    'PID':{'theta':30,'dtheta':0,'pi':10,'control':0}
        }                    
        #self.control = [0,0]
        self.dt = 1/10
        self.dt_list = []
        #ref_상대거리, cur_상대거리
        self.z_PID_list = []
        self.z_ref_list = []
        self.z1_ref_list = []
        
        self.z_ref = 0
        # self.D_SPID_list = []
        # self.D_APID_list = []
        self.x_PID_list = []
        self.x_ref_list = []
        self.x_ref = 0

        self.theta_PID_list = []
        self.theta_ref_list = []
        self.theta_ref = 0


    def V_cal(self):
        global trigger
        trigger += 1

        for key in self.data_z.keys():
            self.data_z[key]['z'] += self.data_z[key]['dz'] * self.dt
            self.data_z[key]['z']

        for key in self.data_x.keys():
            self.data_x[key]['x'] += self.data_x[key]['dx'] * self.dt
            self.data_x[key]['x']


        for key in self.data_theta.keys():
            self.data_theta[key]['theta'] += self.data_theta[key]['dtheta'] * self.dt
            self.data_theta[key]['theta']

        V_control_z = {'PID': self.PID.z_control((self.data_z['PID']['z']) ,202-2*trigger)#,

        #V_control_z = {'PID': self.PID.z_control((self.data_z['PID']['z']) ,self.data_z['goal_z']['z'])#,
        # 'APID': self.APID.Vcontrol((self.data['leader']['D'] - self.data['APID']['D']) ,self.D_ref),'SPID': self.SPID.Vcontrol((self.data['leader']['D'] - self.data['SPID']['D']) ,self.D_ref)
        }
        V_control_x = {'PID': self.PID.x_control((self.data_x['PID']['x']) ,self.data_x['goal_x']['x'])#,
        # 'APID': self.APID.Vcontrol((self.data['leader']['D'] - self.data['APID']['D']) ,self.D_ref),'SPID': self.SPID.Vcontrol((self.data['leader']['D'] - self.data['SPID']['D']) ,self.D_ref)
        }

        V_control_theta = {'PID': self.PID.theta_control((self.data_theta['PID']['theta']) ,self.data_theta['goal_theta']['theta'])#,
        # 'APID': self.APID.Vcontrol((self.data['leader']['D'] - self.data['APID']['D']) ,self.D_ref),'SPID': self.SPID.Vcontrol((self.data['leader']['D'] - self.data['SPID']['D']) ,self.D_ref)
        }        




        for key in self.data_z.keys():
            if key != 'goal_z':

                self.data_z[key]['F_E'] +=V_control_z['PID']
                self.data_z[key]['z']=((self.data_z['PID']['F_E']*math.cos(self.data_theta['PID']['pi'])*math.cos(self.data_theta['PID']['theta'])-\
                    self.data_z['PID']['F_E']*math.sin(self.data_theta['PID']['pi'])*math.sin(self.data_theta['PID']['theta'])-self.data_x['PID']['F_S']*math.sin(self.data_theta['PID']['theta'])-83.3)/8.5)*self.dt*self.dt/2
                self.data_z[key]['F_E'] = max(min(self.data_z[key]['F_E'], 170), -170)
                #self.data_z['PID']['z']=self.data_z['PID']['F_E']
                self.tmp_z=self.data_z['PID']['F_E']
                
        for key in self.data_x.keys():
            if key != 'goal_x':
                self.data_x['PID']['F_S']+=V_control_x['PID']
                self.data_x['PID']['x']=((self.data_z['PID']['F_E']*math.cos(self.data_theta['PID']['pi'])*math.sin(self.data_theta['PID']['theta'])+self.data_z['PID']['F_E']*math.sin(self.data_theta['PID']['pi'])*math.cos(self.data_theta['PID']['theta'])+self.data_x['PID']['F_S']*math.cos(self.data_theta['PID']['theta'])/8.5))*self.dt*self.dt/2
                self.data_x['PID']['F_S'] = max(min(self.data_x['PID']['F_S'], 5), -5)
                self.tmp_x=self.data_x['PID']['F_S']
        

        for key in self.data_theta.keys():
            if key != 'goal_theta':

                self.data_theta['PID']['pi']+=V_control_theta['PID']
                self.data_theta['PID']['theta']+=(-((self.tmp_z)*math.sin(self.data_theta['PID']['pi'])*(0.685+0.1*math.cos(self.data_theta['PID']['pi']))+(0.575*self.tmp_x))/1.34)*self.dt*self.dt/2
                if(trigger>11):
                    self.data_theta['PID']['theta']/=1.03
                self.data_theta['PID']['pi'] = max(min(self.data_theta['PID']['pi'], 10), -10)

        self.dt_list.append(self.dt * trigger)
        if (trigger<100):
            self.z_PID_list.append(self.data_z['PID']['z']+202-2*trigger)
        else:
            self.z_PID_list.append(self.data_z['PID']['z']-0)



        self.x_PID_list.append(self.data_x['PID']['x']-self.data_x['goal_x']['x'])
        self.theta_PID_list.append(self.data_theta['PID']['theta']-self.data_theta['goal_theta']['theta'])

        # self.D_SPID_list.append(self.data['leader']['D']-self.data['SPID']['D']) 
        # self.D_APID_list.append(self.data['leader']['D']-self.data['APID']['D']) # plot을 위한 상대 거리
        
        self.z_ref_list.append(self.z_ref)
        #self.z1_ref_list.append(0.025*np.exp(-(trigger-10)))
        
        if (trigger<100):
            self.z1_ref_list.append(202-2*trigger)
        else:
            self.z1_ref_list.append(0)
        self.x_ref_list.append(self.x_ref) 
        self.theta_ref_list.append(self.theta_ref) 
        #self.SPID_max = max(self.D_SPID_list)
        #self.APID_max = max(self.D_APID_list)
        self.PID_max = max(self.z_PID_list)
        self.PID_max = max(self.x_PID_list)
        self.PID_max = max(self.theta_PID_list)

        plt.subplot(4,1,1)
        plt.plot(self.dt_list, self.z_PID_list, 'b')
        plt.plot(self.dt_list, self.z_ref_list, 'g')
        plt.plot(self.dt_list, self.z1_ref_list, 'r')
        plt.subplot(4,1,2)
        plt.plot(self.dt_list, self.x_PID_list, 'b')
        plt.plot(self.dt_list, self.x_ref_list, 'g')
        plt.subplot(4,1,3)
        plt.plot(self.dt_list, self.theta_PID_list, 'b')
        plt.plot(self.dt_list, self.theta_ref_list, 'g')

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

        return self.data_z['PID']['dz']

C = Control()
file_name = C.gain_string()
f = open(f'csv/{file_name}.csv', 'w')
write_f = csv.writer(f)

while True:
    C.V_cal()
    if trigger == 1:
        write_f.writerow(['Time', 'PID_distance'])#,'APID_distance','SPID_distance'])        
    write_f.writerow([str(trigger), str(C.z_PID_list[trigger-1])])#, str(C.D_APID_list[trigger-1]),str(C.D_APID_list[trigger-1])])
    if trigger == 800:
        write_f.writerow(['C.PID_max',str(C.PID_max)])      
        # write_f.writerow(['C.SPID_max',str(C.SPID_max)])
        # write_f.writerow(['C.APID_max',str(C.APID_max)])
        plt.savefig(f"image/{file_name}.png")
        sys.exit()
        f.close()
plt.show()

# plt.savefig('%s.png'%(B.gain_string()))