from numpy import linalg as LA
from numpy import *
import math
from scipy.spatial.transform import Rotation as R


class dim3Error:
    def __init__(self, sx, sy, sz, tx, ty, tz):
        self.x = sx - tx
        self.y = sy - ty
        self.z = sz - tz


# a control object saves the trajectory to follow and calculates the next step of corresponding rotor speeds
class Control:
    def __init__(self, waypoints, trajectory):
        # mass
        self.m = 1
        # gravitational acceleration
        self.g = 9.81
        # gains
        self.k_p = 1
        self.k_v = 1

        self.x_path = []
        self.y_path = []
        self.z_path = []
        self.phi_path = []

        for i in range(len(trajectory)):
            t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
            self.x_path = hstack((self.x_path , trajectory[i][0] * t ** 4 + trajectory[i][1] * t ** 3 + trajectory[i][2] * t ** 2 + trajectory[i][3] * t + trajectory[i][4]))
            self.y_path = hstack((self.y_path , trajectory[i][5] * t ** 4 + trajectory[i][6] * t ** 3 + trajectory[i][7] * t ** 2 + trajectory[i][8] * t + trajectory[i][9]))
            self.z_path = hstack((self.z_path , trajectory[i][10] * t ** 4 + trajectory[i][11] * t ** 3 + trajectory[i][12] * t ** 2 + trajectory[i][13] * t + trajectory[i][14]))
            self.phi_path = hstack((self.phi_path , trajectory[i][15] * t ** 2 + trajectory[i][16] * t + trajectory[i][17]))


        # save x, y, z velocity functions
        #self.x_dot_path = 4 * trajectory[0] * t ** 3 + 3 * trajectory[1] * t ** 2 + 2 * trajectory[2] * t + trajectory[3]
        #self.y_dot_path = 4 * trajectory[5] * t ** 3 + 3 * trajectory[6] * t ** 2 + 2 * trajectory[7] * t + trajectory[8]
        #self.z_dot_path = 4 * trajectory[10] * t ** 3 + 3 * trajectory[11] * t ** 2 + 2 * trajectory[12] * t + trajectory[13]

        # save x, y, z acceleration functions
        #self.x_ddot_path = 12 * trajectory[0] * t ** 2 + 6 * trajectory[1] * t + 2 * trajectory[2]
        #self.y_ddot_path = 12 * trajectory[5] * t ** 2 + 6 * trajectory[6] * t + 2 * trajectory[7]
        #self.z_ddot_path = 12 * trajectory[10] * t ** 2 + 6 * trajectory[11] * t + 2 * trajectory[12]

    # just send the next position to go to
    def nextUpPD(self, step):
        return self.x_path[step], self.y_path[step], self.z_path[step], self.phi_path[step]

    # calculate next rotor speeds (Mellinger controller)
    def nextUp(self, step, state):
        error_p = dim3Error(state.x, state.y, state.z, self.x_path[step], self.y_path[step], self.z_path[step])
        error_v = dim3Error(state.x_dot, state.y_dot, state.z_dot, self.x_dot_path[step], self.y_dot_path[step], self.z_dot_path[step])

        # TODO: Just about everything
        # print("Moin", step, self.x_path[step], self.y_path[step], self.z_path[step], self.phi_path[step])

    # controller based on Matlab example
    def nextUpEasy(self, step, state, waypoint1):
        acc = array([self.x_ddot_path[step], self.y_ddot_path[step], self.z_ddot_path[step]])
        zb = array(acc/LA.norm(acc))
        toward = math.atan2(state.y - waypoint1.y, state.x - waypoint1.x)
        d = toward - state.phi
        d = ((d+math.pi) % (2*math.pi)) - math.pi
        if abs(d) >= 1/2 * math.pi / (1/0.05):
            d = abs(d)/d * 1/2 * math.pi / (1/0.05)

        yaw = state.phi + d
        xc = array([math.cos(yaw), math.sin(yaw), 0])
        yb = cross(zb,xc)/LA.norm(cross(zb,xc))
        xb = cross(yb,zb)
        rotm = R.from_dcm([[xb[0],yb[0],zb[0]], [xb[1],yb[1],zb[1]], [xb[2],yb[2],zb[2]]])
        u_rot = rotm.as_euler('xyz')

        u_1 = self.m * LA.norm(acc)
        print("Result", u_1, u_rot[0], u_rot[1], u_rot[2])
        # TODO: convert this to results that do something
