#!/usr/bin/env python
import rospy
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from control_msgs.msg import FollowJointTrajectoryActionGoal
from sensor_msgs.msg import JointState

#rc('font', **{'family':'sans-serif':['Helvetica']})

class MeuSubscrib(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/joint_trajectory_action/feedback', FollowJointTrajectoryActionFeedback, self.sub_callback)
        self.joint_data = FollowJointTrajectoryActionFeedback()

        self.subGoal = rospy.Subscriber('/joint_states', JointState, self.sub_callback_goal)
        self.joint_data_goal = FollowJointTrajectoryActionGoal()
        
        self.list = []  
        self.velocity_list =[]           
        self.erro_list = []
        self.aceleration_list = []
        self.effort_list = []
        
              
        rate = rospy.Rate(1)

    def sub_callback_goal(self, msg):
        self.joint_data_goal = msg
        
        self.effort_desired = self.joint_data_goal
        
        self.effort_desired = list(self.effort_desired)
        self.effort_list.append(self.effort_desired)

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
        
        
        
        self.effort_desired = self.joint_data.feedback.desired.effort
        self.effort_desired = list(self.effort_desired)
        self.effort_list.append(self.effort_desired)
        print(self.effort_desired)
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
        plt.rcParams.update({'font.size':20})
        plt.rc('legend',fontsize = 18)
        plt.plot(t, j1, 'g',label=(r'$\theta_1$')) 
        plt.plot(t, self.j2, 'r',label=(r'$\theta_2$'))
        plt.plot(t, self.j3, 'b',label=(r'$\theta_3$'))
        plt.plot(t, self.j4, 'y',label=(r'$\theta_4$'))
        plt.plot(t, self.j5, 'r',label=(r'$\theta_5$'))
        plt.plot(t, self.j6,label=(r'$\theta_6$'))
        plt.grid(True)

        plt.xlabel("Iterations")
        plt.ylabel("rad")
        plt.title("Position of joints")
        plt.legend()
        plt.show()

    def plot_velocity(self):
        self.wl1, self.wl2, self.wl3, self.wl4, self.wl5, self.wl6 = self.get_uni(self.velocity_list)
        j1 = self.j1
        t = np.array(range(len(self.j1)))
        
        plt.plot(t, self.wl1, 'g',label=(r'$\dot{\theta}_1$')) 
        plt.plot(t, self.wl2, 'r',label=(r'$\dot{\theta}_2$'))
        plt.plot(t, self.wl3, 'b',label=(r'$\dot{\theta}_3$'))
        plt.plot(t, self.wl4, 'y',label=(r'$\dot{\theta}_4$'))
        plt.plot(t, self.wl5, 'r',label=(r'$\dot{\theta}_5$'))
        plt.plot(t, self.wl6,label=(r'$\dot{\theta}_6$'))
        
        plt.grid(True)
        plt.xlabel("Iterations")
        plt.ylabel("Rad/s")
        plt.title("Velocity of Joints")
        plt.legend()
        plt.show()

    def plot_erro_position(self):
        self.el1, self.el2, self.el3, self.el4, self.el5, self.el6 = self.get_uni(self.erro_list)
        j1 = self.j1
        t = np.array(range(len(j1)))
        
        plt.plot(t, self.el1, 'g',label=(r'$e_1$')) 
        plt.plot(t, self.el2, 'r',label=(r'$e_2$'))
        plt.plot(t, self.el3, 'b',label=(r'$e_3$'))
        plt.plot(t, self.el4, 'y',label=(r'$e_4$'))   
        plt.plot(t, self.el5, 'r',label=(r'$e_5$'))
        plt.plot(t, self.el6,label=(r'$e_6$'))

        plt.grid(True)
        plt.xlabel("Iterations")
        plt.ylabel("Erro")
        plt.title("Erro of Joints")
        plt.legend()
        plt.show()

    def plot_ace_desired(self):
        self.al1, self.al2, self.al3, self.al4, self.al5, self.al6 = self.get_uni(self.aceleration_list)
        j1 = self.j1
        t = np.array(range(len(j1)))
        
        plt.plot(t, self.al1, 'g',label=(r'$\ddot{\theta}_1$')) 
        plt.plot(t, self.al2, 'r',label=(r'$\ddot{\theta}_2$'))
        plt.plot(t, self.al3, 'b',label=(r'$\ddot{\theta}_3$'))
        plt.plot(t, self.al4, 'y',label=(r'$\ddot{\theta}_4$'))
        plt.plot(t, self.al5, 'r',label=(r'$\ddot{\theta}_5$'))
        plt.plot(t, self.al6,label=(r'$\ddot{\theta}_6$'))

        plt.grid(True)
        plt.xlabel("Iterations")
        plt.ylabel("rad/s2")
        plt.title("Aceleration of Joints")
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
        plt.xlabel("Iterations")
        plt.ylabel("Effort in N.m")
        plt.title("Effort of joint")
        plt.legend()
        plt.show()


if __name__ == '__main__':
    rospy.init_node('simplesub', anonymous=True)
    meusimplesSub = MeuSubscrib()
    rospy.spin()
    lista = meusimplesSub.list
    velocity_list = meusimplesSub.velocity_list    
    aceleration_list = meusimplesSub.aceleration_list
    erro_list = meusimplesSub.erro_list
    effort_list = meusimplesSub.effort_list

    with open('positions.csv', 'w') as file:
        writer = csv.writer(file)
        writer.writerows(lista)
    
    with open('velocity.csv', 'w') as file:
        writerV = csv.writer(file)
        writerV.writerows(velocity_list)

    with open('aceleration.csv', 'w') as file:
        writerA = csv.writer(file)
        writerA.writerows(aceleration_list)    

    with open('erro.csv', 'w') as file:
        writerE = csv.writer(file)
        writerE.writerows(erro_list)    

    with open('effort.csv', 'w') as file:
        writerE = csv.writer(file)
        writerE.writerows(effort_list)    
        

    meusimplesSub.plot_position()
    meusimplesSub.plot_velocity()
    meusimplesSub.plot_erro_position()
    meusimplesSub.plot_ace_desired()
    meusimplesSub.plot_effort_desired()