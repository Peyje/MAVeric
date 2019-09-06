import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from numpy import *
import matplotlib.pyplot as plot


def draw_traj(waypoints, trajectory):
    """
    Visualize the trajectories in every dimension by using matplotlib.

    The code is quite repetitive and might be optimized, but it works...
    """
    mpl.rcParams['legend.fontsize'] = 10

    # =============================
    # 3D Plot
    # =============================
    ax = plot.subplot2grid((23, 31), (0, 0), colspan=13, rowspan=13, projection='3d')  # create Axes3D object, which can plot in 3D
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i+1].time, (waypoints[i+1].time-waypoints[i].time)*20)
        x_path = trajectory[i][0] * t ** 4 + trajectory[i][1] * t ** 3 + trajectory[i][2] * t ** 2 + trajectory[i][3] * t + trajectory[i][4]
        y_path = trajectory[i][5] * t ** 4 + trajectory[i][6] * t ** 3 + trajectory[i][7] * t ** 2 + trajectory[i][8] * t + trajectory[i][9]
        z_path = trajectory[i][10] * t ** 4 + trajectory[i][11] * t ** 3 + trajectory[i][12] * t ** 2 + trajectory[i][13] * t + trajectory[i][14]

        ax.plot(x_path, y_path, z_path, label='[%d] to [%d]' %(i, i+1))  # plot trajectory
        ax.plot([waypoints[i+1].x], [waypoints[i+1].y], [waypoints[i+1].z],'ro')  # plot start
        if i == 0:
            ax.plot([waypoints[i].x], [waypoints[i].y], [waypoints[i].z], 'ro')  # plot end

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # =============================
    # Position Plots
    # =============================
    # add 2D plot of X over time
    ax = plot.subplot2grid((23, 31), (13, 0),  colspan = 6, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i+1].time, (waypoints[i+1].time-waypoints[i].time)*20)
        x_path = trajectory[i][0] * t ** 4 + trajectory[i][1] * t ** 3 + trajectory[i][2] * t ** 2 + trajectory[i][3] * t + trajectory[i][4]
        ax.plot(t, x_path)
    ax.set_ylabel('X')

    # add 2D plot of Y over time
    ax = plot.subplot2grid((23, 31), (19, 0),  colspan = 6, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i+1].time, (waypoints[i+1].time-waypoints[i].time)*20)
        y_path = trajectory[i][5] * t ** 4 + trajectory[i][6] * t ** 3 + trajectory[i][7] * t ** 2 + trajectory[i][8] * t + trajectory[i][9]
        ax.plot(t, y_path)
    ax.set_ylabel('Y')
    ax.set_xlabel('Time')

    # add 2D plot of Z over time
    ax = plot.subplot2grid((23, 31), (13, 7),  colspan = 6, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i+1].time, (waypoints[i+1].time-waypoints[i].time)*20)
        z_path = trajectory[i][10] * t ** 4 + trajectory[i][11] * t ** 3 + trajectory[i][12] * t ** 2 + trajectory[i][13] * t + trajectory[i][14]
        ax.plot(t, z_path)
    ax.set_ylabel('Z')

    # add 2D plot of Yaw over time
    ax = plot.subplot2grid((23, 31), (19, 7), colspan = 6, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i+1].time, (waypoints[i+1].time-waypoints[i].time)*20)
        yaw_path = trajectory[i][15] * t ** 2 + trajectory[i][16] * t + trajectory[i][17]
        ax.plot(t, yaw_path)
    ax.set_ylabel('Yaw')
    ax.set_xlabel('Time')

    # =============================
    # Velocity Plots
    # =============================
    # add 2D plot of X over time
    ax = plot.subplot2grid((23, 31), (0, 15), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        x_path = 4 * trajectory[i][0] * t ** 3 + 3 * trajectory[i][1] * t ** 2 + 2 * trajectory[i][2] * t + trajectory[i][3]
        ax.plot(t, x_path)
    ax.set_ylabel('X')

    # add 2D plot of Y over time
    ax = plot.subplot2grid((23, 31), (6, 15), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        y_path = 4 * trajectory[i][5] * t ** 3 + 3 * trajectory[i][6] * t ** 2 + 2* trajectory[i][7] * t + trajectory[i][8]
        ax.plot(t, y_path)
    ax.set_ylabel('Y')
    ax.set_xlabel('Time')

    # add 2D plot of Z over time
    ax = plot.subplot2grid((23, 31), (0, 19), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        z_path = 4 * trajectory[i][10] * t ** 3 + 3 * trajectory[i][11] * t ** 2 + 2 * trajectory[i][12] * t + trajectory[i][13]
        ax.plot(t, z_path)
    ax.set_ylabel('Z')

    # add 2D plot of Yaw over time
    ax = plot.subplot2grid((23, 31), (6, 19), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        yaw_path = 2 * trajectory[i][15] * t + trajectory[i][16]
        ax.plot(t, yaw_path)
    ax.set_ylabel('Yaw')
    ax.set_xlabel('Time')

    # =============================
    # Acceleration Plots
    # =============================
    # add 2D plot of X over time
    ax = plot.subplot2grid((23, 31), (13, 15), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        x_path = 12 * trajectory[i][0] * t ** 2 + 6 * trajectory[i][1] * t + 2 * trajectory[i][2]
        ax.plot(t, x_path)
    ax.set_ylabel('X')

    # add 2D plot of Y over time
    ax = plot.subplot2grid((23, 31), (19, 15), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        y_path = 12 * trajectory[i][5] * t ** 2 + 6 * trajectory[i][6] * t + 2 * trajectory[i][7]
        ax.plot(t, y_path)
    ax.set_ylabel('Y')
    ax.set_xlabel('Time')

    # add 2D plot of Z over time
    ax = plot.subplot2grid((23, 31), (13, 19), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        z_path = 12 * trajectory[i][10] * t ** 2 + 6 * trajectory[i][11] * t + 2 * trajectory[i][12]
        ax.plot(t, z_path)
    ax.set_ylabel('Z')

    # add 2D plot of Yaw over time
    ax = plot.subplot2grid((23, 31), (19, 19), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        oneVec = linspace(1,1, (waypoints[i + 1].time - waypoints[i].time) * 20)
        yaw_path = 2 * trajectory[i][15] * oneVec
        ax.plot(t, yaw_path)
    ax.set_ylabel('Yaw')
    ax.set_xlabel('Time')

    # =============================
    # Jerk Plots
    # =============================
    # add 2D plot of X over time
    ax = plot.subplot2grid((23, 31), (0, 24), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        x_path = 24 * trajectory[i][0] * t + 6 * trajectory[i][1]
        ax.plot(t, x_path)
    ax.set_ylabel('X')

    # add 2D plot of Y over time
    ax = plot.subplot2grid((23, 31), (6, 24), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        y_path = 24 * trajectory[i][5] * t + 6 * trajectory[i][6]
        ax.plot(t, y_path)
    ax.set_ylabel('Y')
    ax.set_xlabel('Time')

    # add 2D plot of Z over time
    ax = plot.subplot2grid((23, 31), (0, 28), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        z_path = 24 * trajectory[i][10] * t + 6 * trajectory[i][11]
        ax.plot(t, z_path)
    ax.set_ylabel('Z')
    ax.set_xlabel('Time')

    # =============================
    # Snap Plots
    # =============================
    # add 2D plot of X over time
    ax = plot.subplot2grid((23, 31), (13, 24), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        oneVec = linspace(1,1, (waypoints[i + 1].time - waypoints[i].time) * 20)
        x_path = 24 *trajectory[i][0] * oneVec
        ax.plot(t, x_path)
    ax.set_ylabel('X')

    # add 2D plot of Y over time
    ax = plot.subplot2grid((23, 31), (19, 24), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        oneVec = linspace(1,1, (waypoints[i + 1].time - waypoints[i].time) * 20)
        y_path = 24 * trajectory[i][5] * oneVec
        ax.plot(t, y_path)
    ax.set_ylabel('Y')
    ax.set_xlabel('Time')

    # add 2D plot of Z over time
    ax = plot.subplot2grid((23, 31), (13, 28), colspan=3, rowspan=4)
    for i in range(len(trajectory)):
        t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
        oneVec = linspace(1,1, (waypoints[i + 1].time - waypoints[i].time) * 20)
        z_path = 24 * trajectory[i][10] * oneVec
        ax.plot(t, z_path)
    ax.set_ylabel('Z')
    ax.set_xlabel('Time')

    # =============================
    # Labels
    # =============================
    ax = plot.subplot2grid((23, 31), (18, 6))
    ax.set_frame_on(False)
    ax.axis('off')
    ax.text(-0.3,0.7,"Position", fontweight='bold')

    ax = plot.subplot2grid((23, 31), (5, 18))
    ax.set_frame_on(False)
    ax.axis('off')
    ax.text(-0.3,0.7,"Velocity", fontweight='bold')

    ax = plot.subplot2grid((23, 31), (18, 18))
    ax.set_frame_on(False)
    ax.axis('off')
    ax.text(-0.7,0.7,"Acceleration", fontweight='bold')

    ax = plot.subplot2grid((23, 31), (5, 27))
    ax.set_frame_on(False)
    ax.axis('off')
    ax.text(0,0.7,"Jerk", fontweight='bold')

    ax = plot.subplot2grid((23, 31), (18, 27))
    ax.set_frame_on(False)
    ax.axis('off')
    ax.text(-0.1,0.7,"Snap", fontweight='bold')

    #plot.figtext(0, 0, 'Planned Trajectory:\n '
    #                   '(X,Y,Z,Yaw,X_dot,Y_dot,Z_dot)\n '
    #                   'Start: (%0.2f, %0.2f, %0.2f, %0.2f, %0.2f,%0.2f, %0.2f)\n '
    #                   'End: (%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f) \n'
    #                   'Time for segment: %0.2f'
    #             % (waypoint0.x, waypoint0.y, waypoint0.z, waypoint0.yaw, waypoint0.x_dot, waypoint0.y_dot,
    #                waypoint0.z_dot,
    #                waypoint1.x, waypoint1.y, waypoint1.z, waypoint1.yaw, waypoint1.x_dot, waypoint1.y_dot,
    #                waypoint1.z_dot,
    #                waypoint1.time - waypoint0.time))

    # print to screen
    plot.show()
