from numpy import *
from scipy.sparse import csc_matrix
from qpsolvers import solve_qp
import draw

class Waypoint:
    def __init__(self, x, y ,z, phi, x_dot, y_dot, z_dot, time):
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.x_dot = x_dot
        self.y_dot = y_dot
        self.z_dot = z_dot
        self.time = time


def planner(state, traj_x, traj_y, traj_z, traj_phi):
    waypoint0 = Waypoint(state.x, state.y, state.z, state.phi, state.x_dot, state.y_dot, state.z_dot, 0)
    waypoint1 = Waypoint(traj_x, traj_y, traj_z, traj_phi, 0, 0, 0, 5) # TODO: calculate time, don't just assume 5

    # =============================
    # Identity matrix for main part of QP (normally the Hesse matrix, but this is a least squared problem)
    # =============================
    # P_numpy = eye(18)
    P_numpy = zeros((18, 18))  # TODO: Matlab example is not a full identity matrix, why?
    P_numpy[0, 0] = 1
    P_numpy[5, 5] = 1
    P_numpy[10, 10] = 10000
    P_numpy[15, 15] = 1
    P = csc_matrix(P_numpy)  # convert to CSC for performance

    # =============================
    # Gradient vector (linear terms), we have none
    # =============================
    q = zeros((18, 1))
    q = hstack(q)  # convert to hstack for performance

    # =============================
    # Inequality matrix (left side), we have none
    # =============================
    G = zeros((14, 18))
    G = csc_matrix(G)  # convert to CSC for performance

    # =============================
    # Inequality vector (right side), we have none
    # =============================
    h = zeros((14, 1))
    h = hstack(h)  # convert to hstack for performance

    # =============================
    # Equality matrix (left side)
    # =============================
    A = zeros((14, 18))
    # start position and yaw constraints
    A[0, 0] = waypoint0.time ** 4
    A[0, 1] = waypoint0.time ** 3
    A[0, 2] = waypoint0.time ** 2
    A[0, 3] = waypoint0.time
    A[0, 4] = 1
    A[1, 5] = waypoint0.time ** 4
    A[1, 6] = waypoint0.time ** 3
    A[1, 7] = waypoint0.time ** 2
    A[1, 8] = waypoint0.time
    A[1, 9] = 1
    A[2, 10] = waypoint0.time ** 4
    A[2, 11] = waypoint0.time ** 3
    A[2, 12] = waypoint0.time ** 2
    A[2, 13] = waypoint0.time
    A[2, 14] = 1
    A[3, 15] = waypoint0.time ** 2
    A[3, 16] = waypoint0.time
    A[3, 17] = 1
    # end position and yaw constraints
    A[4, 0] = waypoint1.time ** 4
    A[4, 1] = waypoint1.time ** 3
    A[4, 2] = waypoint1.time ** 2
    A[4, 3] = waypoint1.time
    A[4, 4] = 1
    A[5, 5] = waypoint1.time ** 4
    A[5, 6] = waypoint1.time ** 3
    A[5, 7] = waypoint1.time ** 2
    A[5, 8] = waypoint1.time
    A[5, 9] = 1
    A[6, 10] = waypoint1.time ** 4
    A[6, 11] = waypoint1.time ** 3
    A[6, 12] = waypoint1.time ** 2
    A[6, 13] = waypoint1.time
    A[6, 14] = 1
    A[7, 15] = waypoint1.time ** 2
    A[7, 16] = waypoint1.time
    A[7, 17] = 1
    # start velocity constraints
    A[8,0] = 4 * waypoint0.time ** 3
    A[8,1] = 3 * waypoint0.time ** 2
    A[8,2] = 2 * waypoint0.time
    A[8,3] = 1
    A[8,4] = 0
    A[9,5] = 4 * waypoint0.time ** 3
    A[9,6] = 3 * waypoint0.time ** 2
    A[9,7] = 2 * waypoint0.time
    A[9,8] = 1
    A[9,9] = 0
    A[10,10] = 4 * waypoint0.time ** 3
    A[10,11] = 3 * waypoint0.time ** 2
    A[10,12] = 2 * waypoint0.time
    A[10,13] = 1
    A[10,14] = 0
    # end velocity constraints TODO: Like in Matlab example, but why?
    A[11,0] = cos(waypoint1.phi) * 4 * waypoint1.time ** 3
    A[11,1] = cos(waypoint1.phi) * 3 * waypoint1.time ** 2
    A[11,2] = cos(waypoint1.phi) * 2 * waypoint1.time
    A[11,3] = cos(waypoint1.phi)
    A[11,4] = 0
    A[11,5] = sin(waypoint1.phi) * 4 * waypoint1.time ** 3
    A[11,6] = sin(waypoint1.phi) * 3 * waypoint1.time ** 2
    A[11,7] = sin(waypoint1.phi) * 2 * waypoint1.time
    A[11,8] = sin(waypoint1.phi)
    A[11,9] = 0
    A[12,0] = sin(waypoint1.phi) * 4 * waypoint1.time ** 3
    A[12,1] = sin(waypoint1.phi) * 3 * waypoint1.time ** 2
    A[12,2] = sin(waypoint1.phi) * 2 * waypoint1.time
    A[12,3] = sin(waypoint1.phi)
    A[12,4] = 0
    A[12,5] = -1 * cos(waypoint1.phi) * 4 * waypoint1.time ** 3
    A[12,6] = -1 * cos(waypoint1.phi) * 3 * waypoint1.time ** 2
    A[12,7] = -1 * cos(waypoint1.phi) * 2 * waypoint1.time
    A[12,8] = -1 * cos(waypoint1.phi)
    A[12,9] = 0
    A[13,10] = 4 * waypoint1.time ** 3
    A[13,11] = 3 * waypoint1.time ** 2
    A[13,12] = 2 * waypoint1.time
    A[13,13] = 1
    A[13,14] = 0

    A = csc_matrix(A)  # convert to CSC for performance

    # =============================
    # Equality vector (right side)
    # =============================
    b = zeros((14, 1))
    # start position and yaw constraints
    b[0, 0] = waypoint0.x
    b[1, 0] = waypoint0.y
    b[2, 0] = waypoint0.z
    b[3, 0] = waypoint0.phi
    # end position and yaw constraints
    b[4, 0] = waypoint1.x
    b[5, 0] = waypoint1.y
    b[6, 0] = waypoint1.z
    b[7, 0] = waypoint1.phi
    # start velocity constraints
    b[8,0] = waypoint0.x_dot
    b[9,0] = waypoint0.y_dot
    b[10,0] = waypoint0.z_dot
    # end velocity constraints
    b[11,0] = waypoint1.x_dot
    b[12,0] = waypoint1.y_dot
    b[13,0] = waypoint1.z_dot
    b = hstack(b)  # convert to hstack for performance

    trajectory = solve_qp(P, q, G, h, A, b, solver="osqp") # solver = "quadprog" (default), "cvxpy", "osqp"
    print("QP solution:", trajectory )
    draw.draw_traj(waypoint0, waypoint1, trajectory)