
from numpy import *


# a control object saves the trajectory to follow and calculates the next step of corresponding rotor speeds
class Control:
    def __init__(self, waypoints, trajectory):
        self.x_path = []
        self.y_path = []
        self.z_path = []
        self.yaw_path = []

        for i in range(len(trajectory)):
            t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * 20)
            self.x_path = hstack((self.x_path, trajectory[i][0] * t ** 4 + trajectory[i][1] * t ** 3 + trajectory[i][2] * t ** 2 + trajectory[i][3] * t + trajectory[i][4]))
            self.y_path = hstack((self.y_path, trajectory[i][5] * t ** 4 + trajectory[i][6] * t ** 3 + trajectory[i][7] * t ** 2 + trajectory[i][8] * t + trajectory[i][9]))
            self.z_path = hstack((self.z_path, trajectory[i][10] * t ** 4 + trajectory[i][11] * t ** 3 + trajectory[i][12] * t ** 2 + trajectory[i][13] * t + trajectory[i][14]))
            self.yaw_path = hstack((self.yaw_path, trajectory[i][15] * t ** 2 + trajectory[i][16] * t + trajectory[i][17]))

    # just send the next position to go to
    def nextUpPD(self, step):
        return self.x_path[step], self.y_path[step], self.z_path[step], self.yaw_path[step]
