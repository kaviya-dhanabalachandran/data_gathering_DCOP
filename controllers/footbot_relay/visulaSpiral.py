#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv
import matplotlib.patches as mpatches

import matplotlib.cbook as cbook


#fname2 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_probabilitydistribution/positionSpiral.csv"
fname2 = "/home/kavya/workspace/spiralTesting/positionSpiral.csv"
#fname3 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/position3.csv"

#fnameMeeting = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/timestep3.csv"
#fname_goal1 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/goal2.csv"



def getColumn(filename, column):
    results = csv.reader(open(filename), delimiter=",")
    return [result[column] for result in results]


x_0 = getColumn(fname2,0)
y_0 = getColumn(fname2,1)

print np.shape(x_0)
'''

x_1 = getColumn(fname3,0)
y_1 = getColumn(fname3,1)

m_x = getColumn(fnameMeeting,1)
m_y = getColumn(fnameMeeting,2)
'''

#ax = plt.gca()

plt.plot(x_0,y_0,'b--',label="relay1")
#c = mpatches.Circle((x_0[0], y_0[0]), 0.2, facecolor = 'none', edgecolor="red", linewidth=1)

#plt.plot(x_1,y_1,'g--',label="agent1")

#plt.scatter(m_x,m_y,c='k')
#ax.plot(xg_2,yg_2,'o',label="goal agent1")


#ax.add_artist(c)

plt.xlim(0.0,10.0)
plt.ylim(0.0,10.0)

plt.show()