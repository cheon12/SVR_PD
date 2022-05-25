from APID import APID
from PID import PID
from SPID import SPID

import matplotlib.pyplot as plt
from time import sleep
import sys
import csv

trigger = 0
class Control(APID):
    def __init__(self):
        super().__init__()
        #클래스 지정
        self.SPID=SPID()
        self.APID = APID()
        self.PID = PID()
        self.data = {'leader':{'D' :1 , 'V' : 0.3}, 
                    'PID':{'D' : 0, 'V' : 0, 'control': 0},
                    'APID':{'D' : 0, 'V' : 0, 'control' : 0},
                    'SPID':{'D' : 0, 'V' : 0, 'control' : 0},
                   }
        self.control = [0,0]
        self.dt = 1/10
        self.dt_list = []
        
        #ref_상대거리, cur_상대거리
        self.D_SPID_list = []
        self.D_APID_list = []
        self.D_PID_list = []
        self.D_ref_list = []
        self.D_ref = 0
        
    def V_cal(self):
        global trigger
        trigger += 1

        for key in self.data.keys():
            self.data[key]['D'] += self.data[key]['V'] * self.dt

        V_control = {'PID': self.PID.Vcontrol((self.data['leader']['D'] - self.data['PID']['D']) ,self.D_ref),
        'APID': self.APID.Vcontrol((self.data['leader']['D'] - self.data['APID']['D']) ,self.D_ref),'SPID': self.SPID.Vcontrol((self.data['leader']['D'] - self.data['SPID']['D']) ,self.D_ref)
        }
    
        for key in self.data.keys():
            if key != 'leader':
                self.data[key]['control'] = V_control[key]
                self.data[key]['V'] += self.data[key]['control']
                self.data[key]['V'] = max(min(self.data[key]['V'], 0.6), 0)

        self.dt_list.append(self.dt * trigger) 
        
        self.D_SPID_list.append(self.data['leader']['D']-self.data['SPID']['D']) 
        self.D_APID_list.append(self.data['leader']['D']-self.data['APID']['D']) # plot을 위한 상대 거리
        self.D_PID_list.append(self.data['leader']['D']-self.data['PID']['D'])
        
        self.D_ref_list.append(self.D_ref) 
        #self.SPID_max = max(self.D_SPID_list)
        #self.APID_max = max(self.D_APID_list)
        #self.PID_max = max(self.D_PID_list)

        plt.subplot(3,1,1)
        plt.plot(self.dt_list, self.D_PID_list, 'b')
        plt.plot(self.dt_list, self.D_ref_list, 'g')
        plt.subplot(3,1,2)
        plt.plot(self.dt_list, self.D_APID_list, 'b')
        plt.plot(self.dt_list, self.D_ref_list, 'g')
        plt.subplot(3,1,3)
        plt.plot(self.dt_list, self.D_SPID_list, 'b')
        plt.plot(self.dt_list, self.D_ref_list, 'g')
        plt.pause(0.001)

        return self.data['APID']['V']

#속도범위 0.00m/s ~ 0.611m/s 속도 input 0 ~ 800 따라서 계수가 1309
C = Control()
file_name = C.gain_string()
f = open(f'csv/{file_name}.csv', 'w')
write_f = csv.writer(f)

while True:
    C.V_cal()
    if trigger == 1:
        write_f.writerow(['Time', 'PID_distance','APID_distance','SPID_distance'])        
    write_f.writerow([str(trigger), str(C.D_PID_list[trigger-1]), str(C.D_APID_list[trigger-1]),str(C.D_APID_list[trigger-1])])
    if trigger == 200:
        write_f.writerow(['C.SPID_max',str(C.SPID_max)])
        write_f.writerow(['C.APID_max',str(C.APID_max)])
        write_f.writerow(['C.PID_max',str(C.PID_max)])      
        plt.savefig(f"image/{file_name}.png")
        sys.exit()
        f.close()
plt.show()

# plt.savefig('%s.png'%(B.gain_string()))