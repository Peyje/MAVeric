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
    waypoints.append(Waypoint(5, 5, 10, 3, 6))
    waypoints.append(Waypoint(-2, 4, 3, 1, 15))
    waypoints.append(Waypoint(-5, 8, 1, 5, 20))
    waypoints.append(Waypoint(7, -3, 2, 4, 28))


    # Total number of segments
    numSegments = len(waypoints) - 1
    # Every segment has its own polynomial of 4th degree for X,Y and Z and a polynomial of 2nd degree for Phi
    numCoefficients = 3*5+3
    # list of calulated trajectory coefficients
    trajectory = []

    for i in range(numSegments):
        # 2*(3+1) Positional + (4*3)+(2*1) Start Constraints
        numConstraints = 22
        # (4*3)+(2*1) Extra Constraints only for absolute End
        # they are initialized as zero, so no changes needed
        if i == numSegments-1:
            numConstraints += 14

        # =============================
        # Identity matrix for main part of QP (normally the Hesse matrix, but this is a least squared problem)
        # =============================
        P_numpy = zeros((numCoefficients, numCoefficients))
        P_numpy[0, 0] = 1  # minimize snap for X
        P_numpy[5, 5] = 1  # minimize snap for Y
        P_numpy[10, 10] = 1  # minimize snap for Z
        P_numpy[15, 15] = 1  # minimize acceleration for Phi
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
        # G = csc_matrix(G)  # convert to CSC for performance

        # =============================
        # Inequality vector (right side), we have none
        # =============================
        h = zeros((numConstraints, 1))
        h = hstack(h)  # convert to hstack for performance

        # =============================
        # Equality matrix (left side)
        # =============================
        A = zeros((numConstraints, numCoefficients))
        # X Position Start
        A[0,0] = waypoints[i].time ** 4
        A[0,1] = waypoints[i].time ** 3
        A[0,2] = waypoints[i].time ** 2
        A[0,3] = waypoints[i].time
        A[0,4] = 1
        # Y Position Start
        A[1, 5] = waypoints[i].time ** 4
        A[1, 6] = waypoints[i].time ** 3
        A[1, 7] = waypoints[i].time ** 2
        A[1, 8] = waypoints[i].time
        A[1, 9] = 1
        # Z Position Start
        A[2, 10] = waypoints[i].time ** 4
        A[2, 11] = waypoints[i].time ** 3
        A[2, 12] = waypoints[i].time ** 2
        A[2, 13] = waypoints[i].time
        A[2, 14] = 1
        # Phi Angle Start
        A[3, 15] = waypoints[i].time ** 2
        A[3, 16] = waypoints[i].time
        A[3, 17] = 1

        # X Position End
        A[4, 0] = waypoints[i + 1].time ** 4
        A[4, 1] = waypoints[i + 1].time ** 3
        A[4, 2] = waypoints[i + 1].time ** 2
        A[4, 3] = waypoints[i + 1].time
        A[4, 4] = 1
        # Y Position End
        A[5, 5] = waypoints[i + 1].time ** 4
        A[5, 6] = waypoints[i + 1].time ** 3
        A[5, 7] = waypoints[i + 1].time ** 2
        A[5, 8] = waypoints[i + 1].time
        A[5, 9] = 1
        # Z Position End
        A[6, 10] = waypoints[i + 1].time ** 4
        A[6, 11] = waypoints[i + 1].time ** 3
        A[6, 12] = waypoints[i + 1].time ** 2
        A[6, 13] = waypoints[i + 1].time
        A[6, 14] = 1
        # Phi Angle End
        A[7, 15] = waypoints[i + 1].time ** 2
        A[7, 16] = waypoints[i + 1].time
        A[7, 17] = 1

        # X Velocity Rendezvous
        A[8, 0] = 4 * waypoints[i].time ** 3
        A[8, 1] = 3 * waypoints[i].time ** 2
        A[8, 2] = 2 * waypoints[i].time
        A[8, 3] = 1
        # Y Velocity Rendezvous
        A[9, 5] = 4 * waypoints[i].time ** 3
        A[9, 6] = 3 * waypoints[i].time ** 2
        A[9, 7] = 2 * waypoints[i].time
        A[9, 8] = 1
        # Z Velocity Rendezvous
        A[10, 10] = 4 * waypoints[i].time ** 3
        A[10, 11] = 3 * waypoints[i].time ** 2
        A[10, 12] = 2 * waypoints[i].time
        A[10, 13] = 1
        # Phi Velocity Rendezvous
        A[11, 15] = 2 * waypoints[i].time
        A[11, 16] = 1

        # X Acceleration Rendezvous
        A[12, 0] = 12 * waypoints[i].time ** 2
        A[12, 1] = 6 * waypoints[i].time
        A[12, 2] = 2
        # Y Acceleration Rendezvous
        A[13, 5] = 12 * waypoints[i].time ** 2
        A[13, 6] = 6 * waypoints[i].time
        A[13, 7] = 2
        # Z Acceleration Rendezvous
        A[14, 10] = 12 * waypoints[i].time ** 2
        A[14, 11] = 6 * waypoints[i].time
        A[14, 12] = 2
        # Phi Acceleration Rendezvous
        #A[15, 15] = 2

        # X Jerk Rendezvous
        #A[16, 0] = 24 * waypoints[i].time
        #A[16, 1] = 6
        # Y Jerk Rendezvous
        #A[17, 5] = 24 * waypoints[i].time
        #A[17, 6] = 6
        # Z Jerk Rendezvous
        #A[18, 10] = 24 * waypoints[i].time
        #A[18, 11] = 6

        # X Snap Rendezvous
        #A[19, 0] = 24
        # Y Snap Rendezvous
        #A[20, 5] = 24
        # Z Snap Rendezvous
        #A[21, 10] = 24

        # =============================
        # Equality vector (right side)
        # =============================
        b = zeros((numConstraints, 1))

        b[0, 0] = waypoints[i].x
        b[1, 0] = waypoints[i].y
        b[2, 0] = waypoints[i].z
        b[3, 0] = waypoints[i].phi
        b[4, 0] = waypoints[i+1].x
        b[5, 0] = waypoints[i+1].y
        b[6, 0] = waypoints[i+1].z
        b[7, 0] = waypoints[i+1].phi

        # Derivatives = 0 for absolute Start
        if i != 0:
            b[8, 0] = 4 * trajectory[-1][0] * waypoints[i].time ** 3 + 3 * trajectory[-1][1] * waypoints[i].time ** 2 + 2 * trajectory[-1][2] * waypoints[i].time + trajectory[-1][3]
            b[9, 0] = 4 * trajectory[-1][5] * waypoints[i].time ** 3 + 3 * trajectory[-1][6] * waypoints[i].time ** 2 + 2* trajectory[-1][7] * waypoints[i].time + trajectory[-1][8]
            b[10, 0] = 4 * trajectory[-1][10] * waypoints[i].time ** 3 + 3 * trajectory[-1][11] * waypoints[i].time ** 2 + 2 * trajectory[-1][12] * waypoints[i].time + trajectory[-1][13]
            b[11, 0] = 2 * trajectory[-1][15] * waypoints[i].time + trajectory[-1][16]

            b[12, 0] = 12 * trajectory[-1][0] * waypoints[i].time ** 2 + 6 * trajectory[-1][1] * waypoints[i].time + 2 * trajectory[-1][2]
            b[13, 0] = 12 * trajectory[-1][5] * waypoints[i].time ** 2 + 6 * trajectory[-1][6] * waypoints[i].time + 2 * trajectory[-1][7]
            b[14, 0] = 12 * trajectory[-1][10] * waypoints[i].time ** 2 + 6 * trajectory[-1][11] * waypoints[i].time + 2 * trajectory[-1][12]

        # A = csc_matrix(A)  # convert to CSC for performance
        b = hstack(b)  # convert to hstack for performance

        l = -inf * ones(len(h))
        qp_A = vstack([G, A])
        qp_A = csc_matrix(qp_A)
        qp_l = hstack([l, b])
        qp_u = hstack([h, b])

        # Setup solver and solve
        m = osqp.OSQP()
        m.setup(P=P, q=q, A=qp_A, l=qp_l, u=qp_u)
        res = m.solve()

        # Save to trajectory variable
        trajectory.append(res.x)
        print("QP solution Number ", i, "following: ", res.x)

    draw.draw_traj(waypoints, trajectory)
    # trajectory = solve_qp(P, q, G, h, A, b, solver="osqp")  # solver = "quadprog" (default), "cvxpy", "osqp"
    # return waypoint0, waypoint1, trajectory
