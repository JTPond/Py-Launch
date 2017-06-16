from missions import around_the_moon_simple as atms
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

time = 400.0
dt = 0.01

X = np.zeros(int(time/dt))
Y = np.zeros(int(time/dt))
Z = np.zeros(int(time/dt))
color = np.zeros(int(time/dt))

atms.sm.boosters['Booster1'].ignition()
#sm.stabalizers['StabalizerYP'].ignition()
#sm.stabalizers['StabalizerXP'].ignition()

for i in range(int(time/dt)):
    if (i*dt % 1.0 == 0.0):
        X[i] = atms.sm.position[0]
        Y[i] = atms.sm.position[1]
        Z[i] = atms.sm.position[2]
        #print sm.alpha()
    ts = atms.timeStep(dt)
    print atms.sm.a() + atms.gravity()
    #if ts == 'empty':
    color[i] = ((atms.sm.a() + atms.gravity())**2.0).sum()

color = (color)/color.max()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.scatter([atms.moon.position[0]],[atms.moon.position[1]],[atms.moon.position[2]])
ax.scatter(X,Y,Z,c=color,cmap='coolwarm')
#ax.set_xlim3d(0.0,scale)
#ax.set_ylim3d(0.0,scale)
#ax.set_zlim3d(0.0,scale)
#plt.title("Gas in a Box, 20 Particle T=500, Gravity + EM")

#plt.savefig(os.path.join('.','big_R_20N_500t_EM.png'),bbox_inches='tight')
#plt.close()
plt.show()
