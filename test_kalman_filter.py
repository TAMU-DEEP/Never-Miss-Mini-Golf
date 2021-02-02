from filterpy.kalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import math

num_predictions = 101
max_time = 100
speed = -.4
measurement_std = .5
offset = 20

dt = 1.

f = KalmanFilter(dim_x=4, dim_z=2)

#initial state x, y, vx, vy
f.x = np.array([0., 0., 0., 0.])

#state trasition matrix 

f.F = np.array([
    [1., 0., dt, 0.],
    [0., 1., 0., dt],
    [0., 0., 1., 0.],
    [0., 0., 0., 1.],
                ])

#measurement function
f.H = np.array([
    [1.,0.,0.,0.],
    [0.,1.,0.,0.],
                ])

#covaraince matrix
f.P *= 1000.

#measurement noise
f.R *= measurement_std #diagonal_noise

# asign noise??
from filterpy.common import Q_discrete_white_noise
f.Q = Q_discrete_white_noise(dim=4, dt=dt, var=0.13)

##  testing:

#now, lets try and do some prediction
time = np.linspace(0,max_time,(max_time+dt)/dt)

#position is linear scaling of time, like a rolling ball
v = np.asarray([.1, .2])
x = [0., 0.]
x0 = [0., 10.]

def position(v,x0,t):
    return v*t+x0

def linear_extrap(time,px,py,vx,vy):
    pos=np.asarray([px,py])
    speed=np.asarray([vx,vy])
    m = speed
    b = pos-time*m
    return lambda t: m*t +b

true_position_list = []
measurement_list = []
kalman_predictions = []
predicted_final_y = []
for t in time:
    pos = position(v,x0,t)
    noise = np.random.normal(size=2,loc=0, scale=(math.sqrt(measurement_std)/2))
    measurement = pos+noise

    true_position_list.append(pos)
    measurement_list.append(measurement)

    #update kalman filter
    f.predict()
    kalman_predictions.append(f.x)
    f.update([measurement])

    #predict final position
    extrapolator = linear_extrap(t,*f.x)
    final_y = extrapolator(max_time)[1]
    predicted_final_y.append(final_y)

#plot it
fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True)

# make a plot
uncertaint_list = np.full(len(measurement_list),measurement_std)

def unzip_list(list_nD):
    return list(zip(*list_nD))

x,y= unzip_list(true_position_list)
m_x,my = unzip_list(measurement_list)
k_x,k_y,v_x,v_y = unzip_list(kalman_predictions)

axes[0].errorbar(m_x,my,label="measurement",linestyle="",marker=".")
axes[0].plot(k_x,k_y,label="kalman prediction")
axes[0].plot(x,predicted_final_y,label="kalman final prediction")
axes[0].plot(x,y,label="true position", linestyle=":",color='red')
axes[0].legend()
axes[0].set_ylim([-0,60])
plt.show()



'''

uncertainty = np.full(num_predictions,measurement_std)
#add some noise:
measurement = true_position+np.random.normal(scale=uncertainty)



implied_trajectories = []
time_til_end_list = []
predictions = []
for t,x in zip(time,measurement):
    f.predict()
    position, speed = f.x
    f.update(x)
    predictions.append(position)

    #extrapolate to end:
    extrapolation_function = linear_extrap(t,*f.x)
    time_till_end = np.linspace(t,max_time,max_time-t+1)
    time_til_end_list.append(time_till_end)
    implied_trajectories.append(extrapolation_function(time_till_end))

fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True)
# make a plot
axes[0].errorbar(time,measurement,yerr=uncertainty,label="measurement",linestyle="",marker=".")
axes[0].plot(time,predictions,label="kalman prediction")
axes[0].plot(time,true_position,label="true position", linestyle=":",color='red')
axes[0].legend()



import matplotlib as mpl
from matplotlib import cm
from collections import OrderedDict

cmaps = OrderedDict()
evenly_spaced_interval = np.linspace(1, 0, len(time))
colors = [cm.viridis(x) for x in evenly_spaced_interval]

for traj,time_til_end,color in zip(implied_trajectories,time_til_end_list,colors):
    plt.plot(time_til_end,traj, color=color, alpha=.2)
axes[1].plot(time,true_position,label="true position", linestyle=":",color='red')
axes[1].legend()
axes[1].set_ylim([-70,70])
plt.savefig("kalman_filter_demo.png")
plt.show()
'''