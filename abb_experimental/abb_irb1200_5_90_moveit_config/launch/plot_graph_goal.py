#!/usr/bin/env python
import rospy
import csv
import numpy as np
import matplotlib.pyplot as plt
from control_msgs.msg import FollowJointTrajectoryActionFeedback



class MeuSubscrib(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/joint_trajectory_action/feedback', FollowJointTrajectoryActionFeedback, self.sub_callback)
        self.joint_data = FollowJointTrajectoryActionFeedback()
        
        self.list = []  
        self.velocity_list =[]           
        self.erro_list = []
        self.aceleration_list = []
        self.effort_list = []
        
              
        rate = rospy.Rate(1)

    def sub_callback(self, msg):
        self.joint_data = msg
        self.pose = self.joint_data.feedback.actual.positions
        self.pose = list(self.pose)
        self.list.append(self.pose)

        self.vel_desired = self.joint_data.feedback.actual.velocities
        self.vel_desired = list(self.vel_desired)
        self.velocity_list.append(self.vel_desired)
        
        self.erro = self.joint_data.feedback.error.positions
        
        self.erro = list(self.erro)
        self.erro_list.append(self.erro)
        
        self.ace_desired = self.joint_data.feedback.desired.accelerations
        self.ace_desired = list(self.ace_desired)
        self.aceleration_list.append(self.ace_desired)
        timee = self.joint_data.feedback.actual.time_from_start
        print(timee)
        
        self.effort_desired = self.joint_data.feedback.actual.effort
        self.effort_desired = list(self.effort_desired)
        self.effort_list.append(self.effort_desired)
    
    def get_uni(self,list):
        xl1 = []
        xl2 = []
        xl3 = []
        xl4 = []
        xl5 = []
        xl6 = []

        for i in range(0,len(list)):
            x1 = list[i][0]
            xl1.append(x1)
            
            x2 = list[i][1]
            xl2.append(x2)
            
            x3 = list[i][2]
            xl3.append(x3)

            x4 = list[i][3]
            xl4.append(x4)

            x5 = list[i][4]
            xl5.append(x5)
            
            x6 = list[i][5]
            xl6.append(x6)

        return xl1, xl2, xl3, xl4, xl5, xl6    
            
    def plot_position(self):
        self.j1, self.j2, self.j3, self.j4, self.j5, self.j6 = self.get_uni(self.list)
        j1 = self.j1
        
        t = np.array(range(len(j1)))
        
        plt.plot(t, j1, 'g',label='theta1') 
        plt.plot(t, self.j2, 'r',label='theta2')
        plt.plot(t, self.j3, 'b',label='theta3')
        plt.plot(t, self.j4, 'y',label='theta4')
        plt.plot(t, self.j5, 'r',label='theta5')
        plt.plot(t, self.j6,label='theta6')
        plt.grid(True)

        plt.xlabel("Tempo em s")
        plt.ylabel("Deslocamento das juntas em rad")
        plt.title("Posicao das juntas")
        plt.legend()
        plt.show()

    def plot_velocity(self):
        self.wl1, self.wl2, self.wl3, self.wl4, self.wl5, self.wl6 = self.get_uni(self.velocity_list)
        j1 = self.j1
        t = np.array(range(len(self.j1)))
        
        plt.plot(t, self.wl1, 'g',label='dtheta1') 
        plt.plot(t, self.wl2, 'r',label='dtheta2')
        plt.plot(t, self.wl3, 'b',label='dtheta3')
        plt.plot(t, self.wl4, 'y',label='dtheta4')
        plt.plot(t, self.wl5, 'r',label='dtheta5')
        plt.plot(t, self.wl6,label='dtheta6')
        
        plt.grid(True)
        plt.xlabel("Tempo em s")
        plt.ylabel("Velocidade das juntas em rad/s")
        plt.title("Velocidades das juntas")
        plt.legend()
        plt.show()

    def plot_erro_position(self):
        self.el1, self.el2, self.el3, self.el4, self.el5, self.el6 = self.get_uni(self.erro_list)
        j1 = self.j1
        t = np.array(range(len(j1)))
        
        plt.plot(t, self.el1, 'g',label='e1') 
        plt.plot(t, self.el2, 'r',label='e2')
        plt.plot(t, self.el3, 'b',label='e3')
        plt.plot(t, self.el4, 'y',label='e4')   
        plt.plot(t, self.el5, 'r',label='e5')
        plt.plot(t, self.el6,label='e6')

        plt.grid(True)
        plt.xlabel("Tempo em s")
        plt.ylabel("erro")
        plt.title("Erro das juntas")
        plt.legend()
        plt.show()

    def plot_ace_desired(self):
        self.al1, self.al2, self.al3, self.al4, self.al5, self.al6 = self.get_uni(self.aceleration_list)
        j1 = self.j1
        t = np.array(range(len(j1)))
        
        plt.plot(t, self.al1, 'g',label='a1') 
        plt.plot(t, self.al2, 'r',label='a2')
        plt.plot(t, self.al3, 'b',label='a3')
        plt.plot(t, self.al4, 'y',label='a4')
        plt.plot(t, self.al5, 'r',label='a5')
        plt.plot(t, self.al6,label='a6')

        plt.grid(True)
        plt.xlabel("Tempo em s")
        plt.ylabel("aceleracao")
        plt.title("Aceleracao das juntas")
        plt.legend()
        plt.show()

    def plot_effort_desired(self):
        self.tl1, self.tl2, self.tl3, self.tl4, self.tl5, self.tl6 = self.get_uni(self.effort_list)
        j1 = self.j1
        t = np.array(range(len(j1)))
        
        plt.plot(t, self.tl1, 'g',label='t1') 
        plt.plot(t, self.tl2, 'r',label='t2')
        plt.plot(t, self.tl3, 'b',label='t3')
        plt.plot(t, self.tl4, 'y',label='t4')
        plt.plot(t, self.tl5, 'r',label='t5')
        plt.plot(t, self.tl6,label='t6')

        plt.grid(True)
        plt.xlabel("Tempo em s")
        plt.ylabel("Torque")
        plt.title("Torque das juntas")
        plt.legend()
        plt.show()


if __name__ == '__main__':
    rospy.init_node('simplesub', anonymous=True)
    meusimplesSub = MeuSubscrib()
    rospy.spin()
    lista = meusimplesSub.list

    with open('positions.csv', 'w') as file:
        writer = csv.writer(file)
        writer.writerows(lista)
    
    meusimplesSub.plot_position()
    meusimplesSub.plot_velocity()
    meusimplesSub.plot_erro_position()
    meusimplesSub.plot_ace_desired()
    meusimplesSub.plot_effort_desired()