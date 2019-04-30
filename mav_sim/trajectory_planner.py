from numpy import *
from scipy.sparse import csc_matrix
from qpsolvers import solve_qp

class Waypoint:
    def __init__(self, x, y ,z, phi, time):
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.time = time
        self.x_dot = 0. # TODO: not used yet
        self.y_dot = 0. # TODO: not used yet
        self.z_dot = 0. # TODO: not used yet


def planner(state, traj_x, traj_y, traj_z, traj_phi):
    waypoint0 = Waypoint(state.x, state.y, state.z, state.phi, 0)
    waypoint1 = Waypoint(traj_x, traj_y, traj_z, traj_phi, 5) # TODO: calculate time

    # =============================
    # Identity matrix for main part of QP (normally the Hesse matrix, but this is a least squared problem)
    # =============================
    P_numpy = eye(18)
    P = csc_matrix(P_numpy)  # convert to CSC for performance

    # =============================
    # Gradient vector (linear terms), we have none
    # =============================
    q = zeros((18, 1))
    q = hstack(q)  # convert to hstack for performance

    # =============================
    # Inequality matrix (left side), we have none
    # =============================
    G = zeros((8, 18))
    G = csc_matrix(G)  # convert to CSC for performance

    # =============================
    # Inequality vector (right side), we have none
    # =============================
    h = zeros((8, 1))
    h = hstack(h)  # convert to hstack for performance

    # =============================
    # Equality matrix (left side)
    # =============================
    A = zeros((8, 18))
    # start constraints
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
    # end constraints
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
    A = csc_matrix(A)  # convert to CSC for performance

    # =============================
    # Equality vector (right side)
    # =============================
    b = zeros((8, 1))
    # start constraints
    b[0, 0] = waypoint0.x
    b[1, 0] = waypoint0.y
    b[2, 0] = waypoint0.z
    b[3, 0] = waypoint0.phi
    # end constraints
    b[4, 0] = waypoint1.x
    b[5, 0] = waypoint1.y
    b[6, 0] = waypoint1.z
    b[7, 0] = waypoint1.phi
    b = hstack(b)  # convert to hstack for performance

    print("QP solution:", solve_qp(P, q, G, h, A, b, solver="osqp"))  # solver = "quadprog" (default), "cvxpy", "osqp"