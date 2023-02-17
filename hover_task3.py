from quadControlSwarm import *
from swarm_pose_est import *
import numpy as np
import threading
import multiprocessing
import time

class hover():
    def __init__(self):
        drone1IP = '192.168.43.216'
        drone2IP = '192.168.43.156'
        # calling the control class
        self.control1 = quadControl(drone1IP)
        self.control2 = quadControl(drone2IP)

        #calling the camera class
        checksum1 = 0
        checksum2 = 0
        self.pose = camera_pose()
        while checksum1 == 0 or checksum2 == 0:
            
            self.init_position_array = self.pose.getPose()
            checksum1 = sum(self.init_position_array[0,:]) 
            checksum2 = sum(self.init_position_array[1,:])        
        # initial position (same as the initial position detected by camera)
        #drone1
        self.init_pos_x_1 = self.init_position_array[0,0]
        self.init_pos_y_1 = self.init_position_array[0,1]
        self.init_pos_z_1 = self.init_position_array[0,2]

        # desired position (same as the initial position detected by camera)
        # self.x_desired_1 = np.round(self.init_pos_x_1, 2)
        # self.y_desired_1 = np.round(self.init_pos_y_1, 2)
        # self.z_desired_1 = np.round(self.init_pos_z_1, 2) -0.5

        # self.x_desired_1 = -0.2
        # self.y_desired_1 = 0.17
        # self.z_desired_1 = 0.9

        self.x_desired_1 = -0.33
        self.y_desired_1 = 0.1
        self.z_desired_1 = 0.90


        #drone2
        self.init_pos_x_2 = self.init_position_array[1,0]
        self.init_pos_y_2 = self.init_position_array[1,1]
        self.init_pos_z_2 = self.init_position_array[1,2]

        # desired position (same as the initial position detected by camera)
        # self.x_desired_2 = np.round(self.init_pos_x_2, 2)
        # self.y_desired_2 = np.round(self.init_pos_y_2, 2)
        # self.z_desired_2 = np.round(self.init_pos_z_2, 2) -0.5
        # print('desired pos = ', self.x_desired,self.y_desired,self.z_desired)

        self.x_desired_2 = 0.25
        self.y_desired_2 = 0.1
        self.z_desired_2 = 0.90

        # desired velocity
        self.xvel_desired_1 = 0
        self.yvel_desired_1 = 0
        self.zvel_desired_1 = 0
        self.xvel_desired_2 = 0
        self.yvel_desired_2 = 0
        self.zvel_desired_2 = 0

        #start timer
        self.timer = time.time()

        self.abort = 0
    
    def path(self):
        self.position_array = self.pose.getPose()
        t1 = threading.Thread(target=self.control1.posControl, args=(self.position_array[0,:],self.x_desired_1,self.y_desired_1,self.z_desired_1,self.xvel_desired_1,self.yvel_desired_1,self.zvel_desired_1,self.abort,))
        t2 = threading.Thread(target=self.control2.posControl, args=(self.position_array[1,:],self.x_desired_2,self.y_desired_2,self.z_desired_2,self.xvel_desired_2,self.yvel_desired_2,self.zvel_desired_2,self.abort,))
        # self.control1.posControl(self.position_array[0,:],self.x_desired_1,self.y_desired_1,self.z_desired_1,self.xvel_desired_1,self.yvel_desired_1,self.zvel_desired_1,self.abort)
        # self.control2.posControl(self.position_array[1,:],self.x_desired_2,self.y_desired_2,self.z_desired_2,self.xvel_desired_2,self.yvel_desired_2,self.zvel_desired_2,self.abort)
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        # p1 = multiprocessing.Process(target=self.control1.posControl, args=(self.position_array[0,:],self.x_desired_1,self.y_desired_1,self.z_desired_1,self.xvel_desired_1,self.yvel_desired_1,self.zvel_desired_1,self.abort,))
        # p2 = multiprocessing.Process(target=self.control2.posControl, args=(self.position_array[1,:],self.x_desired_2,self.y_desired_2,self.z_desired_2,self.xvel_desired_2,self.yvel_desired_2,self.zvel_desired_2,self.abort,))
        # # self.control1.posControl(self.position_array[0,:],self.x_desired_1,self.y_desired_1,self.z_desired_1,self.xvel_desired_1,self.yvel_desired_1,self.zvel_desired_1,self.abort)
        # # self.control2.posControl(self.position_array[1,:],self.x_desired_2,self.y_desired_2,self.z_desired_2,self.xvel_desired_2,self.yvel_desired_2,self.zvel_desired_2,self.abort)
        # p1.start()
        # p2.start()
        # p1.join()
        # p2.join()
        
    def set_hover_time(self,hover_time=10):
      
        while (time.time() - self.timer) < hover_time: #hover for 10 sec
            try:            
                self.path()            
                
            except KeyboardInterrupt:
                # emergency landing
                self.abort = 1                            
                return

if __name__ == '__main__':

    hover_control = hover()
    
    hover_control.set_hover_time(8) #A1
    hover_control.y_desired_1=hover_control.y_desired_1 - 0.23 #B1
    print("Moving d1 to p2")
    hover_control.set_hover_time(14)

    hover_control.x_desired_2 = hover_control.x_desired_1
    hover_control.y_desired_2 = hover_control.y_desired_1 + 0.23 #A2
    print("Moving d2 to p1")
    hover_control.set_hover_time(18)

    hover_control.x_desired_1=hover_control.x_desired_1 + 0.35 #C1
    print("Moving d1 to p3")
    hover_control.set_hover_time(22)
    hover_control.y_desired_2 = hover_control.y_desired_1 #B2
    print("Moving d2 to p2")
    hover_control.set_hover_time(26)

    hover_control.y_desired_1=hover_control.y_desired_1 + 0.23 #D1
    print("Moving d1 to p4")
    hover_control.set_hover_time(30)
    hover_control.x_desired_2 = hover_control.x_desired_1 #C2
    print("Moving d2 to p3")
    hover_control.set_hover_time(34)
    
    hover_control.x_desired_1=hover_control.x_desired_1 - 0.35
    print("Moving d1 to p1")
    hover_control.set_hover_time(38)
    hover_control.y_desired_2 = hover_control.y_desired_1+0.2
    print("Moving d2 to p4")
    hover_control.set_hover_time(42)

    # Land, disarm and disconnect
    t3=threading.Thread(target=hover_control.control1.posControl,args=(hover_control.position_array[0,:],0,0,0,0,0,0,1,))
    t4=threading.Thread(target=hover_control.control2.posControl,args=(hover_control.position_array[1,:],0,0,0,0,0,0,1,))
    # hover_control.control1.posControl(hover_control.position_array[0,:],0,0,0,0,0,0,1)
    # hover_control.control2.posControl(hover_control.position_array[0,:],0,0,0,0,0,0,1)
    t3.start()
    t4.start()
    t3.join()
    t4.join()

    
