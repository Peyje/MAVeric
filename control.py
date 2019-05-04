from numpy import linalg as LA
import math

class Dim3_error:
    def __init__(self, sx, sy, sz, tx, ty ,tz):
        self.x = sx - tx
        self.y = sy - ty
        self.z = sz - tz

# a control objects saves the trajectory to follow and calculates the next step of corresponding rotor speeds
class Control:
    def __init__(self, trajectory, t):
        # mass of Crazyflie
        self.m = 0.028
        # gravitational mass
        self.g = 9.81
        # gains
        self.k_p = 1
        self.k_v = 1

        # save x, y, z and phi functions
        self.x_path = trajectory[0] * t ** 4 + trajectory[1] * t ** 3 + trajectory[2] * t ** 2 + trajectory[3] * t + trajectory[4]
        self.y_path = trajectory[5] * t ** 4 + trajectory[6] * t ** 3 + trajectory[7] * t ** 2 + trajectory[8] * t + trajectory[9]
        self.z_path = trajectory[10] * t ** 4 + trajectory[11] * t ** 3 + trajectory[12] * t ** 2 + trajectory[13] * t + trajectory[14]
        self.phi_path = trajectory[15] * t ** 2 + trajectory[16] * t + trajectory[17]

        # sae x, y, z velocity functions
        self.x_dot_path = 4 * trajectory[0] * t ** 3 + 3 * trajectory[1] * t ** 2 + 2 * trajectory[2] * t + trajectory[3]
        self.y_dot_path = 4 * trajectory[5] * t ** 3 + 3 * trajectory[6] * t ** 2 + 2 * trajectory[7] * t + trajectory[8]
        self.z_dot_path = 4 * trajectory[10] * t ** 3 + 3 * trajectory[11] * t ** 2 + 2 * trajectory[12] * t + trajectory[13]

        # save x, y, z acceleration functions
        self.x_ddot_path = 12 * trajectory[0] * t ** 2 + 6 * trajectory[1] * t + 2 * trajectory[2]
        self.y_ddot_path = 12 * trajectory[5] * t ** 2 + 6 * trajectory[6] * t + 2 * trajectory[7]
        self.z_ddot_path = 12 * trajectory[10] * t ** 2 + 6 * trajectory[11] * t + 2 * trajectory[12]


    # calculate next rotor speeds (Mellinger controller)
    def nextUp(self, step, state):
        error_p = Dim3_error(state.x, state.y, state.z, self.x_path[step], self.y_path[step], self.z_path[step])
        error_v = Dim3_error(state.x_dot, state.y_dot, state.z_dot, self.x_dot_path[step], self.y_dot_path[step], self.z_dot_path[step])

        # TODO: Just about everything
        print("Moin", step, self.x_path[step], self.y_path[step], self.z_path[step], self.phi_path[step])



    # controller based on Matlab example
    def nextUpEasy(self, step, state, waypoint1):
        acc = [self.x_ddot_path[step], self.y_ddot_path[step], self.z_ddot_path[step]]
        zb = acc/LA.norm(acc)
        toward = math.atan2(state.y - waypoint1.y, state.x - waypoint1.x)
        d = toward - state.phi
        print("Hi")

