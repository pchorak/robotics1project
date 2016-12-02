import numpy as np
import DobotModel

data = np.loadtxt('arm_cal.csv',delimiter=',')
angles = data[:,0:3]
xy_ref = data[:,3:5]*(25.4/5)
xy_arm = np.zeros(xy_ref.shape)

p0t = DobotModel.forward_kinematics(angles[0,:])
xy0 = p0t[0:2]
for k in range(xy_ref.shape[0]):
    p0t = DobotModel.forward_kinematics(angles[k,:])
    xy_arm[k,:] =  p0t[0:2] - xy0

np.savetxt('data.csv',np.hstack((angles,xy_ref,xy_arm)),delimiter=',')
