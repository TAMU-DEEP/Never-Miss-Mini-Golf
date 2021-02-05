'''def calculateKalmanGain(estimateError, measurementError):
    return estimateError/(estimateError + measurementError)

def calculateCurrentEstimate(previousEstimate, kalmanGain, measurement):
    return previousEstimate + kalmanGain*(measurement-previousEstimate)

def calculateEstimateError(kalmanGain, previousEstimateError):
    return (1-kalmanGain)*(previousEstimateError)





initialEstimate = 68
previousEstimate = initialEstimate

initialEstimateError = 2
previousEstimateError = initialEstimateError
 
measurements = [75,71,70,74]
measurementError = 4

for measurement in measurements:
    KG = calculateKalmanGain(previousEstimateError,measurementError)
    currentEstimate = calculateCurrentEstimate(previousEstimate, KG, measurement)
    previousEstimate = currentEstimate
    newEstimateError = calculateEstimateError(KG,previousEstimateError)
    previousEstimateError = newEstimateError
    print(currentEstimate)'''


import numpy as np
from filterpy.kalman import KalmanFilter

f = KalmanFilter(dim_x=4, dim_z=2)

dt = 1

x = 0
y = 0
dx = .5
dy = .8
R_val = 5
fric = 1

f.x = np.array([[x],[y],[dx],[dy]])

f.F = np.array([[1,0,dt,0],
                [0,1,0,dt],
                [0,0,fric,0],
                [0,0,0,fric]])



f.P *= 1000.
f.R *= R_val
from filterpy.common import Q_discrete_white_noise
f.Q = Q_discrete_white_noise(dim=4, dt=dt, var=0.13)


print(f.x)
f.predict()
print(f.x)
f.predict()
f.update([[10,20]])
print(f.x)
print("-----")
f.predict()
print(f.x)

'''
 = KalmanFilter(dim_x=4, dim_z=2)

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

#process noise
from filterpy.common import Q_discrete_white_noise
f.Q = Q_discrete_white_noise(dim=4, dt=dt, var=0.13)

'''




'''class KalmanFilter():
    def __init__(self):

        self.x = 0
        self.y = 0
        self.dx = 0
        self.dy = 0

        self.dt = 1

        self.stateMatrix = np.array([[x],[y],[dx],[dy]])

        self.A = np.array([[1,0,dt,0],
        [0,1,0,dt],
        [0,0,1,0],
        [0,0,0,1]])'''