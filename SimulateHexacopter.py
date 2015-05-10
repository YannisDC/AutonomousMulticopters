"""
A simple example of an animated plot... In 3D!
"""
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from math import *

fig = plt.figure()
axes = p3.Axes3D(fig)

class Hexacopter():
    def __init__(self):
        self.matrix = np.array([[0, 0, 26, -26, 26, -26, 0],[-30, 30, 15, -15, -15, 15, 0],[0, 0, 0, 0, 0, 0, 0],[1, 1, 1, 1, 1, 1, 1]])
        self.new_matrix = np.array([[0, 0, 26, -26, 26, -26, 0],[-30, 30, 15, -15, -15, 15, 0],[0, 0, 0, 0, 0, 0, 0],[1, 1, 1, 1, 1, 1, 1]]) # use if position is measured from start like when using webcam

    def plot_line(self,i,color):
        axes.plot([self.matrix[0][i-1],self.matrix[0][6]],[self.matrix[1][i-1],self.matrix[1][6]],[self.matrix[2][i-1],self.matrix[2][6]],color)

    def set_data_type(self,i,line):
        if i==3-1 or i==5-1:
            line.set_color("r")
        else:
            line.set_color("b")
        line.set_data([[self.matrix[0][i],self.matrix[0][6]],[self.matrix[1][i],self.matrix[1][6]]])
        line.set_3d_properties([self.matrix[2][i],self.matrix[2][6]])
        return line

    def plot_hex(self):
        
        self.plot_line(5,"r")
        self.plot_line(3,"r")
        self.plot_line(1,"b")
        self.plot_line(2,"b")
        self.plot_line(4,"b")
        self.plot_line(6,"b")

    def move(self,x):
        ax, ay, az, tx, ty, tz = radians(x[0]), radians(x[1]), radians(x[2]), x[3], x[4], x[5]

        Rx = np.array([[1,0,0],[0,cos(ax),-sin(ax)],[0,sin(ax),cos(ax)]],float)
        Ry = np.array([[cos(ay),0,sin(ay)],[0,1,0],[-sin(ay),0,cos(ay)]],float)
        Rz = np.array([[cos(az),-sin(az),0],[sin(az),cos(az),0],[0,0,1]],float)
        R = np.dot(Rz,np.dot(Ry,Rx))
        t = np.array([[tx],[ty],[tz]])
        
        #R = np.linalg.inv(R)
        t = np.array([[tx],[ty],[tz]])
        t = np.dot(R,t)
        '''
        t[0][0] *= cos(ay)
        
        t[1][0] *= -sin(az)*cos(az)
        t[2][0] *= -cos(az)*cos(ay)
        print t[0][0]
        '''
        bottom = np.array([[0,0,0,1]])
        Mext = np.concatenate((R,t),axis=1)
        Mext = np.concatenate((Mext,bottom),axis=0)

        ph = np.dot(Mext,self.matrix)
        ph1=np.divide(ph[0],ph[3])
        ph2=np.divide(ph[1],ph[3])
        ph3=np.divide(ph[2],ph[3])
        
        a=np.array([ph1,ph2, ph3])
        self.matrix = np.concatenate((a,[[1,1,1,1,1,1,1]]),axis=0)
        #print self.matrix[0][6],self.matrix[1][6],self.matrix[2][6]

h = Hexacopter()
lines = [axes.plot([0,1],[0,1],[0,1]) for i in range(6)]
print lines

def plot_location(num):
    item = [0,0.5,1,1,0,1]
    h.move(item)
    for index, line in enumerate(lines):
        line = h.set_data_type(index, line[0])
    return lines


'''
b = [0,20,45,50,0,50] 
h.move(b)
h.plot_hex()
'''
# Setting the axes properties
axes.set_xlim3d([-100.0, 100.0])
axes.set_xlabel('X')

axes.set_ylim3d([-100.0, 100.0])
axes.set_ylabel('Y')

axes.set_zlim3d([0.0, 100.0])
axes.set_zlabel('Z')

axes.set_title('3D Flight simulation')

line_ani = animation.FuncAnimation(fig, plot_location, 50,
                              interval=50, blit=False, repeat=False)

line_ani.save('Hexacopter_simulation.mp4', fps=15)

#plt.show()
