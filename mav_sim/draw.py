import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from numpy import *
import matplotlib.pyplot as plot

def draw_traj(waypoint0, waypoint1, trajectory):
    mpl.rcParams['legend.fontsize'] = 10

    t = linspace(waypoint0.time,waypoint1.time,1000)
    x_path = trajectory[0] * t ** 4 + trajectory[1] * t ** 3 + trajectory[2] * t ** 2 + trajectory[3] * t + trajectory[4]
    y_path = trajectory[5] * t ** 4 + trajectory[6] * t ** 3 + trajectory[7] * t ** 2 + trajectory[8] * t + trajectory[9]
    z_path = trajectory[10] * t ** 4 + trajectory[11] * t ** 3 + trajectory[12] * t ** 2 + trajectory[13] * t + trajectory[14]
    phi_path = trajectory[15] * t ** 2 + trajectory[16] * t + trajectory[17]

    # add 3D plot
    ax = plot.subplot2grid((4,7),(0,0), colspan = 3, rowspan = 3,projection='3d') #create Axes3D object, which can plot in 3D
    ax.plot(x_path, y_path, z_path, label='3D Trajectory') # plot trajectory
    ax.plot([waypoint0.x],[waypoint0.y],[waypoint0.z],'ro') # plot start
    ax.plot([waypoint1.x], [waypoint1.y], [waypoint1.z], 'ro') # plot end
    plot.figtext(0,0,'Planned Trajectory:\n (X,Y,Z,Phi,X_dot,Y_dot,Z_dot)\n Start: (%0.2f, %0.2f, %0.2f, %0.2f, %0.2f,%0.2f, %0.2f)\n End: (%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)' \
                 %(waypoint0.x, waypoint0.y, waypoint0.z, waypoint0.phi, waypoint0.x_dot,waypoint0.y_dot,waypoint0.z_dot,waypoint1.x, waypoint1.y, waypoint1.z, waypoint1.phi, waypoint1.x_dot,waypoint1.y_dot,waypoint1.z_dot))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # add 2D plot of X over time
    ax = plot.subplot2grid((4,7),(0,4), colspan = 3)
    ax.plot(t, x_path)
    ax.set_ylabel('X')
    # add 2D plot of Y over time
    ax = plot.subplot2grid((4,7),(1,4), colspan = 3)
    ax.plot(t, y_path)
    ax.set_ylabel('Y')
    # add 2D plot of Z over time
    ax = plot.subplot2grid((4,7),(2,4), colspan = 3)
    ax.plot(t, z_path)
    ax.set_ylabel('Z')
    # add 2D plot of Phi over time
    ax = plot.subplot2grid((4,7),(3,4), colspan = 3)
    ax.plot(t, phi_path)
    ax.set_ylabel('Phi')
    ax.set_xlabel('Time')

    # print to screen
    plot.show()