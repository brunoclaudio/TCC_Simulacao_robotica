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
        self.vel_desired_list = []
        self.j1 = []
        self.j2 = []
        self.j3 = []
        self.j4 = []
        self.j5 = []
        self.j6 = []
        
        self.wl1 = []
        self.wl2 = []
        self.wl3 = []
        self.wl4 = []
        self.wl5 = []
        self.wl6 = []
        
        self.al1 = []
        self.al2 = []
        self.al3 = []
        self.al4 = []
        self.al5 = []
        self.al6 = []
        
        self.el1 = []
        self.el2 = []
        self.el3 = []
        self.el4 = []
        self.el5 = []
        self.el6 = []
        
        self.tl1 = []
        self.tl2 = []
        self.tl3 = []
        self.tl4 = []
        self.tl5 = []
        self.tl6 = []
        
        self.segundos = []
        rate = rospy.Rate(1)

    def get_poses(self):
        jj1 = self.pose[0]
        jj2 = self.pose[1]
        jj3 = self.pose[2]
        jj4 = self.pose[3]
        jj5 = self.pose[4]
        jj6 = self.pose[5]
        return jj1, jj2, jj3, jj4, jj5, jj6

    def get_velocites_desired(self):
        w1 = self.vel_desired[0]
        w2 = self.vel_desired[1]
        w3 = self.vel_desired[2]
        w4 = self.vel_desired[3]
        w5 = self.vel_desired[4]
        w6 = self.vel_desired[5]
        return w1, w2, w3, w4, w5, w6

    def get_erro(self):
        e1 = self.erro[0]
        e2 = self.erro[1]
        e3 = self.erro[2]
        e4 = self.erro[3]
        e5 = self.erro[4]
        e6 = self.erro[5]
        return e1, e2, e3, e4, e5, e6

    def get_ace_desired(self):
        a1 = self.ace_desired[0]
        a2 = self.ace_desired[1]
        a3 = self.ace_desired[2]
        a4 = self.ace_desired[3]
        a5 = self.ace_desired[4]
        a6 = self.ace_desired[5]
        return a1, a2, a3, a4, a5, a6

    def get_effort_desired(self):
        t1 = self.ace_desired[0]
        t2 = self.ace_desired[1]
        t3 = self.ace_desired[2]
        t4 = self.ace_desired[3]
        t5 = self.ace_desired[4]
        t6 = self.ace_desired[5]
        return t1, t2, t3, t4, t5, t6      

    def sub_callback(self, msg):
        self.joint_data = msg
        self.pose = self.joint_data.feedback.desired.positions
        self.pose = list(self.pose)
        jj1, jj2, jj3, jj4, jj5, jj6 = self.get_poses()

        self.vel_desired = self.joint_data.feedback.desired.velocities
        self.vel_desired = list(self.vel_desired)
        w1, w2, w3, w4, w5, w6 = self.get_velocites_desired()

        self.erro = self.joint_data.feedback.error.positions
        self.erro = list(self.erro)
        e1, e2, e3, e4, e5, e6 = self.get_erro()

        self.ace_desired = self.joint_data.feedback.desired.accelerations
        self.ace_desired = list(self.ace_desired)
        a1, a2, a3, a4, a5, a6 = self.get_ace_desired()
           
        self.effort_desired = self.joint_data.feedback.desired.effort
        self.effort_desired = list(self.effort_desired)
        t1, t2, t3, t4, t5, t6 = self.get_effort_desired()
        #self.jj1 = self.pose[0]
        self.j1.append(jj1)
        self.j2.append(jj2)
        self.j3.append(jj3)
        self.j4.append(jj4)
        self.j5.append(jj5)
        self.j6.append(jj6)

        self.wl1.append(w1)
        self.wl2.append(w2)
        self.wl3.append(w3)
        self.wl4.append(w4)
        self.wl5.append(w5)
        self.wl6.append(w6)
        
        self.el1.append(e1)
        self.el2.append(e2)
        self.el3.append(e3)
        self.el4.append(e4)
        self.el5.append(e5)
        self.el6.append(e6)

        self.al1.append(a1)
        self.al2.append(a2)
        self.al3.append(a3)
        self.al4.append(a4)
        self.al5.append(a5)
        self.al6.append(a6)

        self.tl1.append(t1)
        self.tl2.append(t2)
        self.tl3.append(t3)
        self.tl4.append(t4)
        self.tl5.append(t5)
        self.tl6.append(t6)

        self.list.append(self.pose)
        print(self.j1)
            
    def plot_position(self):
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