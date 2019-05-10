from numpy import *
from scipy.sparse import csc_matrix
from qpsolvers import solve_qp
import draw
import osqp

class Waypoint:
    def __init__(self, x, y, z, phi, time):
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi

        self.time = time


if __name__ == "__main__":
    waypoints = []
    waypoints.append(Waypoint(0, 0, 2, 0, 0))
    waypoints.append(Waypoint(5, 5, 10, 3, 5))
    waypoints.append(Waypoint(-2, 5, 3, 1, 10))

    numSegments = len(waypoints) - 1
    # Every segment has its own polynomial of 4th degree for X,Y and Z and a polynomial of 2nd degree for Phi
    numCoefficients = numSegments * (3*5+3)
    # For Start/End: 18 Constraints
    # For each Middle Waypoint: 4 + 4 + 14
    numConstraints = 18 + 18 + (numSegments-1) * 22


    # =============================
    # Identity matrix for main part of QP (normally the Hesse matrix, but this is a least squared problem)
    # =============================
    P_numpy = zeros((numCoefficients, numCoefficients))
    for i in range(numSegments):
        P_numpy[0+i*18, 0+i*18] = 1 # minimize snap for X
        P_numpy[5+i*18, 5+i*18] = 1 # minimize snap for Y
        P_numpy[10+i*18, 10+i*18] = 1 # minimize snap for Z
        P_numpy[15+i*18, 15+i*18] = 1 # minimize acceleration for Phi
    P = csc_matrix(P_numpy)  # convert to CSC for performance

    # =============================
    # Gradient vector (linear terms), we have none
    # =============================
    q = zeros((numCoefficients, 1))
    q = hstack(q)  # convert to hstack for performance

    # =============================
    # Inequality matrix (left side), we have none
    # =============================
    G = zeros((numConstraints, numCoefficients))
    #G = csc_matrix(G)  # convert to CSC for performance

    # =============================
    # Inequality vector (right side), we have none
    # =============================
    h = zeros((numConstraints, 1))
    h = hstack(h)  # convert to hstack for performance

    # =============================
    # Equality matrix (left side)
    # =============================
    A = zeros((numConstraints,numCoefficients))

    # =============================
    # Equality vector (right side)
    # =============================
    b = zeros((numConstraints, 1))


    # =============================
    # Set up of Equality Constraints
    # =============================
    cc = -1 # Current Constraint
    for i in range(numSegments):
        # "start of segment" position constraints
        cc += 1 # X Position
        A[cc, 0 + i * 18] = waypoints[i].time ** 4
        A[cc, 1 + i * 18] = waypoints[i].time ** 3
        A[cc, 2 + i * 18] = waypoints[i].time ** 2
        A[cc, 3 + i * 18] = waypoints[i].time
        A[cc, 4 + i * 18] = 1
        b[cc,0] = waypoints[i].x
        cc += 1 # Y Position
        A[cc, 5 + i * 18] = waypoints[i].time ** 4
        A[cc, 6 + i * 18] = waypoints[i].time ** 3
        A[cc, 7 + i * 18] = waypoints[i].time ** 2
        A[cc, 8 + i * 18] = waypoints[i].time
        A[cc, 9 + i * 18] = 1
        b[cc, 0] = waypoints[i].y
        cc += 1 # Z Position
        A[cc, 10 + i * 18] = waypoints[i].time ** 4
        A[cc, 11 + i * 18] = waypoints[i].time ** 3
        A[cc, 12 + i * 18] = waypoints[i].time ** 2
        A[cc, 13 + i * 18] = waypoints[i].time
        A[cc, 14 + i * 18] = 1
        b[cc, 0] = waypoints[i].z
        cc += 1 # Phi Angle
        A[cc, 15 + i * 18] = waypoints[i].time ** 2
        A[cc, 16 + i * 18] = waypoints[i].time
        A[cc, 17 + i * 18] = 1
        b[cc, 0] = waypoints[i].phi

        # "end of segment" position constraints
        cc += 1 # X Position
        A[cc, 0 + i * 18] = waypoints[i + 1].time ** 4
        A[cc, 1 + i * 18] = waypoints[i + 1].time ** 3
        A[cc, 2 + i * 18] = waypoints[i + 1].time ** 2
        A[cc, 3 + i * 18] = waypoints[i + 1].time
        A[cc, 4 + i * 18] = 1
        b[cc, 0] = waypoints[i+1].x
        cc += 1 # Y Position
        A[cc, 5 + i * 18] = waypoints[i + 1].time ** 4
        A[cc, 6 + i * 18] = waypoints[i + 1].time ** 3
        A[cc, 7 + i * 18] = waypoints[i + 1].time ** 2
        A[cc, 8 + i * 18] = waypoints[i + 1].time
        A[cc, 9 + i * 18] = 1
        b[cc, 0] = waypoints[i+1].y
        cc += 1 # Z Position
        A[cc, 10 + i * 18] = waypoints[i + 1].time ** 4
        A[cc, 11 + i * 18] = waypoints[i + 1].time ** 3
        A[cc, 12 + i * 18] = waypoints[i + 1].time ** 2
        A[cc, 13 + i * 18] = waypoints[i + 1].time
        A[cc, 14 + i * 18] = 1
        b[cc, 0] = waypoints[i+1].z
        cc += 1 # Phi Angle
        A[cc, 15 + i * 18] = waypoints[i + 1].time ** 2
        A[cc, 16 + i * 18] = waypoints[i + 1].time
        A[cc, 17 + i * 18] = 1
        b[cc, 0] = waypoints[i+1].phi

        # segment rendezvous constraints
        if i == 0:
            continue

        cc += 1 # X Velocity Rendezvous
        A[cc, 0 + i * 18] = 4 * waypoints[i].time ** 3
        A[cc, 1 + i * 18] = 3 * waypoints[i].time ** 2
        A[cc, 2 + i * 18] = 2 * waypoints[i].time
        A[cc, 3 + i * 18] = 1
        A[cc, 0 + i * 18 - 18] = -1 * A[cc, 0 + i * 18]
        A[cc, 1 + i * 18 - 18] = -1 * A[cc, 1 + i * 18]
        A[cc, 2 + i * 18 - 18] = -1 * A[cc, 2 + i * 18]
        A[cc, 3 + i * 18 - 18] = -1 * A[cc, 3 + i * 18]
        cc += 1 # Y Velocity Rendezvous
        A[cc, 5 + i * 18] = 4 * waypoints[i].time ** 3
        A[cc, 6 + i * 18] = 3 * waypoints[i].time ** 2
        A[cc, 7 + i * 18] = 2 * waypoints[i].time
        A[cc, 8 + i * 18] = 1
        A[cc, 5 + i * 18 - 18] = -1 * A[cc, 5 + i * 18]
        A[cc, 6 + i * 18 - 18] = -1 * A[cc, 6 + i * 18]
        A[cc, 7 + i * 18 - 18] = -1 * A[cc, 7 + i * 18]
        A[cc, 8 + i * 18 - 18] = -1 * A[cc, 8 + i * 18]
        cc += 1 # Z Velocity Rendezvous
        A[cc, 10 + i * 18] = 4 * waypoints[i].time ** 3
        A[cc, 11 + i * 18] = 3 * waypoints[i].time ** 2
        A[cc, 12 + i * 18] = 2 * waypoints[i].time
        A[cc, 13 + i * 18] = 1
        A[cc, 10 + i * 18 - 18] = -1 * A[cc, 10 + i * 18]
        A[cc, 11 + i * 18 - 18] = -1 * A[cc, 11 + i * 18]
        A[cc, 12 + i * 18 - 18] = -1 * A[cc, 12 + i * 18]
        A[cc, 13 + i * 18 - 18] = -1 * A[cc, 13 + i * 18]
        cc += 1 # Phi Velocity Rendezvous
        A[cc, 15 + i * 18] = 2 * waypoints[i].time
        A[cc, 16 + i * 18] = 1
        A[cc, 15 + i * 18 - 18] = -1 * A[cc, 15 + i * 18]
        A[cc, 16 + i * 18 - 18] = -1 * A[cc, 16 + i * 18]


        cc += 1 # X Acceleration Rendezvous
        A[cc, 0 + i * 18] = 12 * waypoints[0].time ** 2
        A[cc, 1 + i * 18] = 6 * waypoints[0].time
        A[cc, 2 + i * 18] = 2
        A[cc, 0 + i * 18 - 18] = -1 * A[cc, 0 + i * 18]
        A[cc, 1 + i * 18 - 18] = -1 * A[cc, 1 + i * 18]
        A[cc, 2 + i * 18 - 18] = -1 * A[cc, 2 + i * 18]
        cc += 1  # Y Acceleration Rendezvous
        A[cc, 5 + i * 18] = 12 * waypoints[0].time ** 2
        A[cc, 6 + i * 18] = 6 * waypoints[0].time
        A[cc, 7 + i * 18] = 2
        A[cc, 5 + i * 18 - 18] = -1 * A[cc, 5 + i * 18]
        A[cc, 6 + i * 18 - 18] = -1 * A[cc, 6 + i * 18]
        A[cc, 7 + i * 18 - 18] = -1 * A[cc, 7 + i * 18]
        cc += 1  # Z Acceleration Rendezvous
        A[cc, 10 + i * 18] = 12 * waypoints[0].time ** 2
        A[cc, 11 + i * 18] = 6 * waypoints[0].time
        A[cc, 12 + i * 18] = 2
        A[cc, 10 + i * 18 - 18] = -1 * A[cc, 10 + i * 18]
        A[cc, 11 + i * 18 - 18] = -1 * A[cc, 11 + i * 18]
        A[cc, 12 + i * 18 - 18] = -1 * A[cc, 12 + i * 18]
        cc += 1  # Phi Acceleration Rendezvous
        A[cc, 15 + i * 18] = 2
        A[cc, 15 + i * 18 - 18] = -1 * A[cc, 15 + i * 18]

        cc += 1 # X Jerk Rendezvous
        A[cc, 0] = 24 * waypoints[0].time
        A[cc, 1] = 6
        A[cc, 0 + i * 18 - 18] = -1 * A[cc, 0 + i * 18]
        A[cc, 1 + i * 18 - 18] = -1 * A[cc, 1 + i * 18]
        cc += 1  # Y Jerk Rendezvous
        A[cc, 5] = 24 * waypoints[0].time
        A[cc, 6] = 6
        A[cc, 5 + i * 18 - 18] = -1 * A[cc, 5 + i * 18]
        A[cc, 6 + i * 18 - 18] = -1 * A[cc, 6 + i * 18]
        cc += 1  # Z Jerk Rendezvous
        A[cc, 10] = 24 * waypoints[0].time
        A[cc, 11] = 6
        A[cc, 10 + i * 18 - 18] = -1 * A[cc, 10 + i * 18]
        A[cc, 11 + i * 18 - 18] = -1 * A[cc, 11 + i * 18]


        cc += 1 # X Snap Rendezvous
        A[cc, 0] = 24
        A[cc, 0 + i * 18 - 18] = -1 * A[cc, 0 + i * 18]
        cc += 1  # Y Snap Rendezvous
        A[cc, 5] = 24
        A[cc, 5 + i * 18 - 18] = -1 * A[cc, 5 + i * 18]
        cc += 1  # Z Snap Rendezvous
        A[cc, 10] = 24
        A[cc, 10 + i * 18 - 18] = -1 * A[cc, 10 + i * 18]





    cc += 1
    # start velocity constraints
    #A[cc, 0] = 4 * waypoints[0].time ** 3
    #A[cc, 1] = 3 * waypoints[0].time ** 2
    #A[cc, 2] = 2 * waypoints[0].time
    #A[cc, 3] = 1
    #A[cc, 4] = 0
    #A[cc+1, 5] = 4 * waypoints[0].time ** 3
    #A[cc+1, 6] = 3 * waypoints[0].time ** 2
    #A[cc+1, 7] = 2 * waypoints[0].time
    #A[cc+1, 8] = 1
    #A[cc+1, 9] = 0
    #A[cc+2, 10] = 4 * waypoints[0].time ** 3
    #A[cc+2, 11] = 3 * waypoints[0].time ** 2
    #A[cc+2, 12] = 2 * waypoints[0].time
    #A[cc+2, 13] = 1
    #A[cc+2, 14] = 0
    #A[cc+3, 15] = 2 * waypoints[0].time
    #A[cc+3, 16] = 1
    #A[cc+3, 17] = 0

    # end velocity constraints
    #A[cc+4, numCoefficients - 18 + 0] = 4 * waypoints[-1].time ** 3
    #A[cc+4, numCoefficients - 18 + 1] = 3 * waypoints[-1].time ** 2
    #A[cc+4, numCoefficients - 18 + 2] = 2 * waypoints[-1].time
    #A[cc+4, numCoefficients - 18 + 3] = 1
    #A[cc+4, numCoefficients - 18 + 4] = 0
    #A[cc+5, numCoefficients - 18 + 5] = 4 * waypoints[-1].time ** 3
    #A[cc+5, numCoefficients - 18 + 6] = 3 * waypoints[-1].time ** 2
    #A[cc+5, numCoefficients - 18 + 7] = 2 * waypoints[-1].time
    #A[cc+5, numCoefficients - 18 + 8] = 1
    #A[cc+5, numCoefficients - 18 + 9] = 0
    #A[cc+6, numCoefficients - 18 + 10] = 4 * waypoints[-1].time ** 3
    #A[cc+6, numCoefficients - 18 + 11] = 3 * waypoints[-1].time ** 2
    #A[cc+6, numCoefficients - 18 + 12] = 2 * waypoints[-1].time
    #A[cc+6, numCoefficients - 18 + 13] = 1
    #A[cc+6, numCoefficients - 18 + 14] = 0
    #A[cc+7, numCoefficients - 18 + 15] = 2 * waypoints[-1].time
    #A[cc+7, numCoefficients - 18 + 16] = 1
    #A[cc+7, numCoefficients - 18 + 17] = 0

    # start acceleration constraints
    #A[cc+8, 0] = 12 * waypoints[0].time ** 2
    #A[cc+8, 1] = 6 * waypoints[0].time
    #A[cc+8, 2] = 2
    #A[cc+8, 3] = 0
    #A[cc+8, 4] = 0
    #A[cc+9, 5] = 12 * waypoints[0].time ** 2
    #A[cc+9, 6] = 6 * waypoints[0].time
    #A[cc+9, 7] = 2
    #A[cc+9, 8] = 0
    #A[cc+9, 9] = 0
    #A[cc+10, 10] = 12 * waypoints[0].time ** 2
    #A[cc+10, 11] = 6 * waypoints[0].time
    #A[cc+10, 12] = 2
    #A[cc+10, 13] = 0
    #A[cc+10, 14] = 0
    #A[cc+11, 15] = 2
    #A[cc+11, 16] = 0
    #A[cc+11, 17] = 0

    # end acceleration constraints
    #A[cc+12, numCoefficients - 18 + 0] = 12 * waypoints[-1].time ** 2
    #A[cc+12, numCoefficients - 18 + 1] = 6 * waypoints[-1].time
    #A[cc+12, numCoefficients - 18 + 2] = 2
    #A[cc+12, numCoefficients - 18 + 3] = 0
    #A[cc+12, numCoefficients - 18 + 4] = 0
    #A[cc+13, numCoefficients - 18 + 5] = 12 * waypoints[-1].time ** 2
    #A[cc+13, numCoefficients - 18 + 6] = 6 * waypoints[-1].time
    #A[cc+13, numCoefficients - 18 + 7] = 2
    #A[cc+13, numCoefficients - 18 + 8] = 0
    #A[cc+13, numCoefficients - 18 + 9] = 0
    #A[cc+14, numCoefficients - 18 + 10] = 12 * waypoints[-1].time ** 2
    #A[cc+14, numCoefficients - 18 + 11] = 6 * waypoints[-1].time
    #A[cc+14, numCoefficients - 18 + 12] = 2
    #A[cc+14, numCoefficients - 18 + 13] = 0
    #A[cc+14, numCoefficients - 18 + 14] = 0
    #A[cc+15, numCoefficients - 18 + 15] = 2
    #A[cc+15, numCoefficients - 18 + 16] = 0
    #A[cc+15, numCoefficients - 18 + 17] = 0

    # start jerk constraints
    #A[cc+16, 0] = 24 * waypoints[0].time
    #A[cc+16, 1] = 6
    #A[cc+16, 2] = 0
    #A[cc+16, 3] = 0
    #A[cc+16, 4] = 0
    #A[cc+17, 5] = 24 * waypoints[0].time
    #A[cc+17, 6] = 6
    #A[cc+17, 7] = 0
    #A[cc+17, 8] = 0
    #A[cc+17, 9] = 0
    #A[cc+18, 10] = 24 * waypoints[0].time
    #A[cc+18, 11] = 6
    #A[cc+18, 12] = 0
    #A[cc+18, 13] = 0
    #A[cc+18, 14] = 0

    # end jerk constraints
    #A[cc+19, numCoefficients - 18 + 0] = 24 * waypoints[-1].time
    #A[cc+19, numCoefficients - 18 + 1] = 6
    #A[cc+19, numCoefficients - 18 + 2] = 0
    #A[cc+19, numCoefficients - 18 + 3] = 0
    #A[cc+19, numCoefficients - 18 + 4] = 0
    #A[cc+20, numCoefficients - 18 + 5] = 24 * waypoints[-1].time
    #A[cc+20, numCoefficients - 18 + 6] = 6
    #A[cc+20, numCoefficients - 18 + 7] = 0
    #A[cc+20, numCoefficients - 18 + 8] = 0
    #A[cc+20, numCoefficients - 18 + 9] = 0
    #A[cc+21, numCoefficients - 18 + 10] = 24 * waypoints[-1].time
    #A[cc+21, numCoefficients - 18 + 11] = 6
    #A[cc+21, numCoefficients - 18 + 12] = 0
    #A[cc+21, numCoefficients - 18 + 13] = 0
    #A[cc+21, numCoefficients - 18 + 14] = 0

    # start snap constraints
    #A[cc+22, 0] = 24
    #A[cc+22, 1] = 0
    #A[cc+22, 2] = 0
    #A[cc+22, 3] = 0
    #A[cc+22, 4] = 0
    #A[cc+23, 5] = 24
    #A[cc+23, 6] = 0
    #A[cc+23, 7] = 0
    #A[cc+23, 8] = 0
    #A[cc+23, 9] = 0
    #A[cc+24, 10] = 24
    #A[cc+24, 11] = 0
    #A[cc+24, 12] = 0
    #A[cc+24, 13] = 0
    #A[cc+24, 14] = 0

    # end snap constraints
    #A[cc+25, numCoefficients - 18 + 0] = 24
    #A[cc+25, numCoefficients - 18 + 1] = 0
    #A[cc+25, numCoefficients - 18 + 2] = 0
    #A[cc+25, numCoefficients - 18 + 3] = 0
    #A[cc+25, numCoefficients - 18 + 4] = 0
    #A[cc+26, numCoefficients - 18 + 5] = 24
    #A[cc+26, numCoefficients - 18 + 6] = 0
    #A[cc+26, numCoefficients - 18 + 7] = 0
    #A[cc+26, numCoefficients - 18 + 8] = 0
    #A[cc+26, numCoefficients - 18 + 9] = 0
    #A[cc+27, numCoefficients - 18 + 10] = 24
    #A[cc+27, numCoefficients - 18 + 11] = 0
    #A[cc+27, numCoefficients - 18 + 12] = 0
    #A[cc+27, numCoefficients - 18 + 13] = 0
    #A[cc+27, numCoefficients - 18 + 14] = 0




    #A = csc_matrix(A)  # convert to CSC for performance
    b = hstack(b)  # convert to hstack for performance

    #trajectory = solve_qp(P, q, G, h, A, b, solver="osqp")  # solver = "quadprog" (default), "cvxpy", "osqp"
    l = -inf * ones(len(h))
    qp_A = vstack([G, A])
    qp_A = csc_matrix(qp_A)
    qp_l = hstack([l, b])
    qp_u = hstack([h, b])
    m = osqp.OSQP()
    m.setup(P=P, q=q, A=qp_A, l=qp_l, u=qp_u)
    res = m.solve()
    print("QP solution:", res.x)
    #draw.draw_traj(waypoint0, waypoint1, trajectory)
    #return waypoint0, waypoint1, trajectory
