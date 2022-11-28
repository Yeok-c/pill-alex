import numpy as np

z_offset = 0.05
# GO BACK TO CAMERA POSITION
# THIS IS WRONG BECAUSE THIS IS JOINT ANGLES NOT CARTESIAN
P_HOME = np.array([0.075, -0.245, 0.28, 0.00032001149681460804, -0.9998324819174745, 0.01829231174002348, 0.0005450014594178438])
P_OFFSET = np.array([0, 0, z_offset, 0, 0, 0, 0])

## GRASP
P_GRASP = np.array([0.08338970755211737, -0.38056472494047144, 0.1606959209601539, 0.00032001149681460804, -0.9998324819174745, 0.01829231174002348, 0.0005450014594178438])
P_GRASP_ABOVE = P_GRASP + P_OFFSET # Go to above grasping position first

## PLACE ON PLATE
P_PLACE = np.array([0.07527888398880958, -0.28993198184034424, 0.23033367897001927, 0.0003226884309160187, -0.9999996130325639, -0.0006070149194710236, 0.0005489442472058018])
P_PLACE_ABOVE = P_PLACE + P_OFFSET # Go to above placing position first


spline_1 = np.stack((P_HOME, P_GRASP_ABOVE, P_GRASP)).T
print("grasp()")
spline_2 = np.stack((P_GRASP, P_GRASP_ABOVE, P_PLACE_ABOVE, P_PLACE)).T
print("release()")
spline_3 = np.stack((P_PLACE, P_PLACE_ABOVE, P_HOME)).T

# phi = np.linspace(0, 2.*np.pi, 40)
# r = 0.5 + np.cos(phi)         # polar coords
# x, y, z =  r * np.cos(phi), r * np.cos(phi), r * np.sin(phi)    # convert to cartesian

from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# x, y, z = np.array(spline_1[:3,:])
# tck, u = splprep([x, y, z], s=0.1)
# new_points = splev(u, tck)

# ax = plt.axes(projection='3d')
# ax.plot3D(x, y, z, 'ro')
# ax.plot3D(new_points[0], new_points[1], new_points[2], 'r-')

x, y, z = np.array(spline_2[:3,:])
tck, u = splprep([x, y, z], s=10, k = 2)
new_points = splev(u, tck)

ax = plt.axes(projection='3d')
ax.plot3D(x, y, z, 'bo')
ax.plot3D(new_points[0], new_points[1], new_points[2], 'b-')


# x, y, z = np.array(spline_3[:3,:])
# tck, u = splprep([x, y, z], s=0.1)
# new_points = splev(u, tck)

# ax = plt.axes(projection='3d')
# ax.plot3D(x, y, z, 'go')
# ax.plot3D(new_points[0], new_points[1], new_points[2], 'g-')




plt.show()