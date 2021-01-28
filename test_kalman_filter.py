from filterpy.kalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt

num_predictions = 101
max_time = 100
speed = -.4
measurement_std = 5
offset = 20

f = KalmanFilter (dim_x=2, dim_z=1)

#initial state
f.x = np.array([0., 0.])

#state trasition matrix 
f.F = np.array([[1.,1.],
                [0.,1.]])

#measurement function
f.H = np.array([[1.,0.]])

#covaraince matrix
f.P *= 1000.
f.P = np.array([[1000.,    0.],
                [   0., 1000.] ])

#measurement noise
f.R = measurement_std
#f.R = np.array([[5.]])

# asign noise??
from filterpy.common import Q_discrete_white_noise
f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)

#now, lets try and do some prediction
time = np.linspace(0,max_time,max_time+1)
#position is linear scaling of time, like a rolling ball
true_position = speed*time + offset
uncertainty = np.full(num_predictions,measurement_std)
#add some noise:
measurement = true_position+np.random.normal(scale=uncertainty)

def linear_extrap(time,position,speed):
    m = speed
    b = position-time*m
    return lambda t: m*t +b

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
axes[0].errorbar(time,measurement,yerr=np.full(101,5),label="measurement",linestyle="",marker=".")
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

plt.show()
plt.savefig("kalman_filter_demo.png")