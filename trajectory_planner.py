from numpy import *
from scipy.sparse import csc_matrix
import draw
import osqp
import math

class Waypoint:
    def __init__(self, x, y, z, yaw, time):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

        self.time = time

# calculates a fitting time for the trajectory by linking it to the 3d distance
def calc_time(start, end):
    distance3D = sqrt((start[1]-end[1])**2 + (start[2]-end[2])**2 + (start[3]-end[3])**2)
    time = distance3D  # ..where an engineer cries and a programmer sees an easy solution
    return time

def joint(waypoints):
    # total number of segments
    numSegments = len(waypoints) - 1
    # every segment has its own polynomial of 4th degree for X,Y and Z and a polynomial of 2nd degree for Yaw
    numCoefficients = numSegments * (3*5+3)
    # list of calculated trajectory coefficients
    trajectory = []
    # start + end X,Y,Z,Yaw position for every segment: 8
    # rendezvous X,Y,Z,Yaw velocity: 4
    # absolute start + end X,Y,Z (+ start Yaw) velocity: 7
    numConstraints = numSegments * 8 + (numSegments - 1) * 4 + 7

    P_numpy = zeros((numCoefficients, numCoefficients))
    for i in range(numSegments):
        P_numpy[0 + i * 18, 0 + i * 18] = 1  # minimize snap for X
        # P_numpy[2 + i * 18, 2 + i * 18] = 100  # minimize acceleration for X
        P_numpy[5 + i * 18, 5 + i * 18] = 1  # minimize snap for Y
        # P_numpy[7 + i * 18, 7 + i * 18] = 100  # minimize acceleration for Y
        P_numpy[10 + i * 18, 10 + i * 18] = 1  # minimize snap for Z
        # P_numpy[12 + i * 18, 12 + i * 18] = 100  # minimize acceleration for Z
        P_numpy[15 + i * 18, 15 + i * 18] = 1  # minimize acceleration for Yaw
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

    # =============================
    # Inequality vector (right side), we have none
    # =============================
    h = zeros((numConstraints, 1))
    h = hstack(h)  # convert to hstack for performance

    # =============================
    # Equality matrix (left side)
    # =============================
    A = zeros((numConstraints, numCoefficients))

    # =============================
    # Equality vector (right side)
    # =============================
    b = zeros((numConstraints, 1))

    # =============================
    # Set up of Equality Constraints
    # =============================
    cc = -1  # Current Constraint
    for i in range(numSegments):
        # "start of segment" position constraints
        cc += 1  # X Position
        A[cc, 0 + i * 18] = waypoints[i].time ** 4
        A[cc, 1 + i * 18] = waypoints[i].time ** 3
        A[cc, 2 + i * 18] = waypoints[i].time ** 2
        A[cc, 3 + i * 18] = waypoints[i].time
        A[cc, 4 + i * 18] = 1
        b[cc, 0] = waypoints[i].x
        cc += 1  # Y Position
        A[cc, 5 + i * 18] = waypoints[i].time ** 4
        A[cc, 6 + i * 18] = waypoints[i].time ** 3
        A[cc, 7 + i * 18] = waypoints[i].time ** 2
        A[cc, 8 + i * 18] = waypoints[i].time
        A[cc, 9 + i * 18] = 1
        b[cc, 0] = waypoints[i].y
        cc += 1  # Z Position
        A[cc, 10 + i * 18] = waypoints[i].time ** 4
        A[cc, 11 + i * 18] = waypoints[i].time ** 3
        A[cc, 12 + i * 18] = waypoints[i].time ** 2
        A[cc, 13 + i * 18] = waypoints[i].time
        A[cc, 14 + i * 18] = 1
        b[cc, 0] = waypoints[i].z
        cc += 1  # Yaw Angle
        A[cc, 15 + i * 18] = waypoints[i].time ** 2
        A[cc, 16 + i * 18] = waypoints[i].time
        A[cc, 17 + i * 18] = 1
        b[cc, 0] = waypoints[i].yaw

        # "end of segment" position constraints
        cc += 1  # X Position
        A[cc, 0 + i * 18] = waypoints[i + 1].time ** 4
        A[cc, 1 + i * 18] = waypoints[i + 1].time ** 3
        A[cc, 2 + i * 18] = waypoints[i + 1].time ** 2
        A[cc, 3 + i * 18] = waypoints[i + 1].time
        A[cc, 4 + i * 18] = 1
        b[cc, 0] = waypoints[i + 1].x
        cc += 1  # Y Position
        A[cc, 5 + i * 18] = waypoints[i + 1].time ** 4
        A[cc, 6 + i * 18] = waypoints[i + 1].time ** 3
        A[cc, 7 + i * 18] = waypoints[i + 1].time ** 2
        A[cc, 8 + i * 18] = waypoints[i + 1].time
        A[cc, 9 + i * 18] = 1
        b[cc, 0] = waypoints[i + 1].y
        cc += 1  # Z Position
        A[cc, 10 + i * 18] = waypoints[i + 1].time ** 4
        A[cc, 11 + i * 18] = waypoints[i + 1].time ** 3
        A[cc, 12 + i * 18] = waypoints[i + 1].time ** 2
        A[cc, 13 + i * 18] = waypoints[i + 1].time
        A[cc, 14 + i * 18] = 1
        b[cc, 0] = waypoints[i + 1].z
        cc += 1  # Yaw Angle
        A[cc, 15 + i * 18] = waypoints[i + 1].time ** 2
        A[cc, 16 + i * 18] = waypoints[i + 1].time
        A[cc, 17 + i * 18] = 1
        b[cc, 0] = waypoints[i + 1].yaw

        # segment rendezvous constraints
        if i == 0:
            continue

        cc += 1  # X Velocity Rendezvous
        A[cc, 0 + i * 18] = 4 * waypoints[i].time ** 3
        A[cc, 1 + i * 18] = 3 * waypoints[i].time ** 2
        A[cc, 2 + i * 18] = 2 * waypoints[i].time
        A[cc, 3 + i * 18] = 1
        A[cc, 0 + i * 18 - 18] = -1 * A[cc, 0 + i * 18]
        A[cc, 1 + i * 18 - 18] = -1 * A[cc, 1 + i * 18]
        A[cc, 2 + i * 18 - 18] = -1 * A[cc, 2 + i * 18]
        A[cc, 3 + i * 18 - 18] = -1 * A[cc, 3 + i * 18]
        cc += 1  # Y Velocity Rendezvous
        A[cc, 5 + i * 18] = 4 * waypoints[i].time ** 3
        A[cc, 6 + i * 18] = 3 * waypoints[i].time ** 2
        A[cc, 7 + i * 18] = 2 * waypoints[i].time
        A[cc, 8 + i * 18] = 1
        A[cc, 5 + i * 18 - 18] = -1 * A[cc, 5 + i * 18]
        A[cc, 6 + i * 18 - 18] = -1 * A[cc, 6 + i * 18]
        A[cc, 7 + i * 18 - 18] = -1 * A[cc, 7 + i * 18]
        A[cc, 8 + i * 18 - 18] = -1 * A[cc, 8 + i * 18]
        cc += 1  # Z Velocity Rendezvous
        A[cc, 10 + i * 18] = 4 * waypoints[i].time ** 3
        A[cc, 11 + i * 18] = 3 * waypoints[i].time ** 2
        A[cc, 12 + i * 18] = 2 * waypoints[i].time
        A[cc, 13 + i * 18] = 1
        A[cc, 10 + i * 18 - 18] = -1 * A[cc, 10 + i * 18]
        A[cc, 11 + i * 18 - 18] = -1 * A[cc, 11 + i * 18]
        A[cc, 12 + i * 18 - 18] = -1 * A[cc, 12 + i * 18]
        A[cc, 13 + i * 18 - 18] = -1 * A[cc, 13 + i * 18]
        cc += 1  # Yaw Velocity Rendezvous
        A[cc, 15 + i * 18] = 2 * waypoints[i].time
        A[cc, 16 + i * 18] = 1
        A[cc, 15 + i * 18 - 18] = -1 * A[cc, 15 + i * 18]
        A[cc, 16 + i * 18 - 18] = -1 * A[cc, 16 + i * 18]

        # cc += 1  # X Acceleration Rendezvous
        # A[cc, 0 + i * 18] = 12 * waypoints[0].time ** 2
        # A[cc, 1 + i * 18] = 6 * waypoints[0].time
        # A[cc, 2 + i * 18] = 2
        # A[cc, 0 + i * 18 - 18] = -1 * A[cc, 0 + i * 18]
        # A[cc, 1 + i * 18 - 18] = -1 * A[cc, 1 + i * 18]
        # A[cc, 2 + i * 18 - 18] = -1 * A[cc, 2 + i * 18]
        # cc += 1  # Y Acceleration Rendezvous
        # A[cc, 5 + i * 18] = 12 * waypoints[0].time ** 2
        # A[cc, 6 + i * 18] = 6 * waypoints[0].time
        # A[cc, 7 + i * 18] = 2
        # A[cc, 5 + i * 18 - 18] = -1 * A[cc, 5 + i * 18]
        # A[cc, 6 + i * 18 - 18] = -1 * A[cc, 6 + i * 18]
        # A[cc, 7 + i * 18 - 18] = -1 * A[cc, 7 + i * 18]
        # cc += 1  # Z Acceleration Rendezvous
        # A[cc, 10 + i * 18] = 12 * waypoints[0].time ** 2
        # A[cc, 11 + i * 18] = 6 * waypoints[0].time
        # A[cc, 12 + i * 18] = 2
        # A[cc, 10 + i * 18 - 18] = -1 * A[cc, 10 + i * 18]
        # A[cc, 11 + i * 18 - 18] = -1 * A[cc, 11 + i * 18]
        # A[cc, 12 + i * 18 - 18] = -1 * A[cc, 12 + i * 18]
        # cc += 1  # Yaw Acceleration Rendezvous
        # A[cc, 15 + i * 18] = 2
        # A[cc, 15 + i * 18 - 18] = -1 * A[cc, 15 + i * 18]

        # cc += 1  # X Jerk Rendezvous
        # A[cc, 0] = 24 * waypoints[0].time
        # A[cc, 1] = 6
        # A[cc, 0 + i * 18 - 18] = -1 * A[cc, 0 + i * 18]
        # A[cc, 1 + i * 18 - 18] = -1 * A[cc, 1 + i * 18]
        # cc += 1  # Y Jerk Rendezvous
        # A[cc, 5] = 24 * waypoints[0].time
        # A[cc, 6] = 6
        # A[cc, 5 + i * 18 - 18] = -1 * A[cc, 5 + i * 18]
        # A[cc, 6 + i * 18 - 18] = -1 * A[cc, 6 + i * 18]
        # cc += 1  # Z Jerk Rendezvous
        # A[cc, 10] = 24 * waypoints[0].time
        # A[cc, 11] = 6
        # A[cc, 10 + i * 18 - 18] = -1 * A[cc, 10 + i * 18]
        # A[cc, 11 + i * 18 - 18] = -1 * A[cc, 11 + i * 18]
        #
        # cc += 1  # X Snap Rendezvous
        # A[cc, 0] = 24
        # A[cc, 0 + i * 18 - 18] = -1 * A[cc, 0 + i * 18]
        # cc += 1  # Y Snap Rendezvous
        # A[cc, 5] = 24
        # A[cc, 5 + i * 18 - 18] = -1 * A[cc, 5 + i * 18]
        # cc += 1  # Z Snap Rendezvous
        # A[cc, 10] = 24
        # A[cc, 10 + i * 18 - 18] = -1 * A[cc, 10 + i * 18]

    cc += 1 # absolute start X velocity
    A[cc, 0] = 4 * waypoints[0].time ** 3
    A[cc, 1] = 3 * waypoints[0].time ** 2
    A[cc, 2] = 2 * waypoints[0].time
    A[cc, 3] = 1
    cc += 1  # absolute start Y velocity
    A[cc, 5] = 4 * waypoints[0].time ** 3
    A[cc, 6] = 3 * waypoints[0].time ** 2
    A[cc, 7] = 2 * waypoints[0].time
    A[cc, 8] = 1
    cc += 1  # absolute start Z velocity
    A[cc, 10] = 4 * waypoints[0].time ** 3
    A[cc, 11] = 3 * waypoints[0].time ** 2
    A[cc, 12] = 2 * waypoints[0].time
    A[cc, 13] = 1
    cc += 1  # absolute start Yaw velocity
    A[cc, 15] = 2 * waypoints[0].time
    A[cc, 16] = 1

    cc += 1 # absolute end X velocity
    A[cc, numCoefficients - 18 + 0] = 4 * waypoints[-1].time ** 3
    A[cc, numCoefficients - 18 + 1] = 3 * waypoints[-1].time ** 2
    A[cc, numCoefficients - 18 + 2] = 2 * waypoints[-1].time
    A[cc, numCoefficients - 18 + 3] = 1
    cc += 1  # absolute end Y velocity
    A[cc, numCoefficients - 18 + 5] = 4 * waypoints[-1].time ** 3
    A[cc, numCoefficients - 18 + 6] = 3 * waypoints[-1].time ** 2
    A[cc, numCoefficients - 18 + 7] = 2 * waypoints[-1].time
    A[cc, numCoefficients - 18 + 8] = 1
    cc += 1  # absolute end Z velocity
    A[cc, numCoefficients - 18 + 10] = 4 * waypoints[-1].time ** 3
    A[cc, numCoefficients - 18 + 11] = 3 * waypoints[-1].time ** 2
    A[cc, numCoefficients - 18 + 12] = 2 * waypoints[-1].time
    A[cc, numCoefficients - 18 + 13] = 1
    #cc += 1  # absolute end Yaw velocity
    #A[cc, numCoefficients - 18 + 15] = 2 * waypoints[-1].time
    #A[cc, numCoefficients - 18 + 16] = 1

    #cc += 1 # absolute start X acceleration
    # A[c, 0] = 12 * waypoints[0].time ** 2
    # A[c, 1] = 6 * waypoints[0].time
    # A[c, 2] = 2
    #cc += 1  # absolute start Y acceleration
    # A[c, 5] = 12 * waypoints[0].time ** 2
    # A[c, 6] = 6 * waypoints[0].time
    # A[c, 7] = 2
    #cc += 1  # absolute start Z acceleration
    # A[cc, 10] = 12 * waypoints[0].time ** 2
    # A[cc, 11] = 6 * waypoints[0].time
    # A[cc, 12] = 2
    #cc += 1  # absolute start Yaw acceleration
    # A[cc, 15] = 2

    #cc += 1 # absolute end X acceleration
    # A[cc, numCoefficients - 18 + 0] = 12 * waypoints[-1].time ** 2
    # A[cc, numCoefficients - 18 + 1] = 6 * waypoints[-1].time
    # A[cc, numCoefficients - 18 + 2] = 2
    #cc += 1  # absolute end Y acceleration
    # A[cc, numCoefficients - 18 + 5] = 12 * waypoints[-1].time ** 2
    # A[cc, numCoefficients - 18 + 6] = 6 * waypoints[-1].time
    # A[cc, numCoefficients - 18 + 7] = 2
    #cc += 1  # absolute end Z acceleration
    # A[cc, numCoefficients - 18 + 10] = 12 * waypoints[-1].time ** 2
    # A[cc, numCoefficients - 18 + 11] = 6 * waypoints[-1].time
    # A[cc, numCoefficients - 18 + 12] = 2
    #cc += 1  # absolute end Yaw acceleration
    # A[cc, numCoefficients - 18 + 15] = 2

    #cc += 1 # absolute start X jerk
    # A[cc, 0] = 24 * waypoints[0].time
    # A[cc, 1] = 6
    #cc += 1  # absolute start Y jerk
    # A[cc, 5] = 24 * waypoints[0].time
    # A[cc, 6] = 6
    #cc += 1  # absolute start Z jerk
    # A[cc, 10] = 24 * waypoints[0].time
    # A[cc, 11] = 6

    #cc += 1 # absolute end X jerk
    # A[cc, numCoefficients - 18 + 0] = 24 * waypoints[-1].time
    # A[cc, numCoefficients - 18 + 1] = 6
    #cc += 1  # absolute end Y jerk
    # A[cc, numCoefficients - 18 + 5] = 24 * waypoints[-1].time
    # A[cc, numCoefficients - 18 + 6] = 6
    #cc += 1  # absolute end Z jerk
    # A[cc, numCoefficients - 18 + 10] = 24 * waypoints[-1].time
    # A[cc, numCoefficients - 18 + 11] = 6

    #cc += 1 # absolute start X snap
    # A[cc, 0] = 24
    #cc += 1  # absolute start Y snap
    # A[cc, 5] = 24
    #cc += 1  # absolute start Z snap
    # A[cc, 10] = 24

    #cc += 1 # absolute end X snap
    # A[cc, numCoefficients - 18 + 0] = 24
    #cc += 1  # absolute end Y snap
    # A[cc, numCoefficients - 18 + 5] = 24
    #cc += 1  # absolute end Z snap
    # A[cc, numCoefficients - 18 + 10] = 24

    # =============================
    # Solver Setup
    # =============================
    # OSQP needs:
    # P = quadratic terms
    # q = linear terms
    # A = constraint matrix of ALL constraints (inequality & equality)
    # l = lower constraints
    # u = upper constraints
    P = csc_matrix(P)
    q = hstack(q)
    h = hstack(h)
    b = hstack(b)

    A = vstack([G, A])
    A = csc_matrix(A)
    l = -inf * ones(len(h))
    l = hstack([l, b])
    u = hstack([h, b])

    # setup solver and solve
    m = osqp.OSQP()
    m.setup(P=P, q=q, A=A, l=l, u=u)  # extra solver variables can be set here
    res = m.solve()

    # save to trajectory variable
    for i in range(0, size(res.x), 18):
        segment = res.x[i:i + 18]
        trajectory.append(segment)
    print("QP solution Number following: ", res.x)
    return trajectory




def separate(waypoints):
    # every segment has its own polynomial of 4th degree for X,Y and Z and a polynomial of 2nd degree for Yaw
    numCoefficients = 3*5+3
    # total number of segments
    numSegments = len(waypoints) - 1
    # list of calculated trajectory coefficients
    trajectory = []


    for i in range(numSegments):
        # X,Y,Z,Yaw position at start and end: 8
        # X,Y,Z,Yaw velocity at start: 4
        # X,Y,Z acceleration at start: 3
        numConstraints = 15
        # X,Y,Z velocity at absolute end: 3
        # they are initialized as zero, so no changes needed
        if i == numSegments-1:
            numConstraints += 3

        # =============================
        # Identity matrix for main part of QP (normally the Hesse matrix (quadratic terms), but this is a least squared problem)
        # =============================
        P = zeros((numCoefficients, numCoefficients))
        P[0, 0] = 1  # minimize snap for X
        P[5, 5] = 1  # minimize snap for Y
        P[10, 10] = 1  # minimize snap for Z
        P[15, 15] = 1  # minimize acceleration for Yaw

        # =============================
        # Gradient vector (linear terms), we have none
        # =============================
        q = zeros((numCoefficients, 1))

        # =============================
        # Inequality matrix (left side), we have none
        # =============================
        G = zeros((numConstraints, numCoefficients))

        # =============================
        # Inequality vector (right side), we have none
        # =============================
        h = zeros((numConstraints, 1))

        # =============================
        # Equality matrix (left side)
        # =============================
        A = zeros((numConstraints, numCoefficients))
        # X Position Start
        A[0, 0] = waypoints[i].time ** 4
        A[0, 1] = waypoints[i].time ** 3
        A[0, 2] = waypoints[i].time ** 2
        A[0, 3] = waypoints[i].time
        A[0, 4] = 1
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
        # Yaw Angle Start
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
        # Yaw Angle End
        A[7, 15] = waypoints[i + 1].time ** 2
        A[7, 16] = waypoints[i + 1].time
        A[7, 17] = 1

        # X Velocity Start
        A[8, 0] = 4 * waypoints[i].time ** 3
        A[8, 1] = 3 * waypoints[i].time ** 2
        A[8, 2] = 2 * waypoints[i].time
        A[8, 3] = 1
        # Y Velocity Start
        A[9, 5] = 4 * waypoints[i].time ** 3
        A[9, 6] = 3 * waypoints[i].time ** 2
        A[9, 7] = 2 * waypoints[i].time
        A[9, 8] = 1
        # Z Velocity Start
        A[10, 10] = 4 * waypoints[i].time ** 3
        A[10, 11] = 3 * waypoints[i].time ** 2
        A[10, 12] = 2 * waypoints[i].time
        A[10, 13] = 1
        # Yaw Velocity Start
        A[11, 15] = 2 * waypoints[i].time
        A[11, 16] = 1

        # X Acceleration Start
        A[12, 0] = 12 * waypoints[i].time ** 2
        A[12, 1] = 6 * waypoints[i].time
        A[12, 2] = 2
        # Y Acceleration Start
        A[13, 5] = 12 * waypoints[i].time ** 2
        A[13, 6] = 6 * waypoints[i].time
        A[13, 7] = 2
        # Z Acceleration Start
        A[14, 10] = 12 * waypoints[i].time ** 2
        A[14, 11] = 6 * waypoints[i].time
        A[14, 12] = 2
        # Yaw Acceleration Start
        #A[15, 15] = 2

        # X Jerk Start
        #A[16, 0] = 24 * waypoints[i].time
        #A[16, 1] = 6
        # Y Jerk Start
        #A[17, 5] = 24 * waypoints[i].time
        #A[17, 6] = 6
        # Z Jerk Start
        #A[18, 10] = 24 * waypoints[i].time
        #A[18, 11] = 6

        # X Snap Start
        #A[19, 0] = 24
        # Y Snap Start
        #A[20, 5] = 24
        # Z Snap Start
        #A[21, 10] = 24

        # for full stop at absolute End
        if i == numSegments - 1:
            # X Velocity End
            A[15, 0] = 4 * waypoints[i + 1].time ** 3
            A[15, 1] = 3 * waypoints[i + 1].time ** 2
            A[15, 2] = 2 * waypoints[i + 1].time
            A[15, 3] = 1
            # Y Velocity End
            A[16, 5] = 4 * waypoints[i + 1].time ** 3
            A[16, 6] = 3 * waypoints[i + 1].time ** 2
            A[16, 7] = 2 * waypoints[i + 1].time
            A[16, 8] = 1
            # Z Velocity End
            A[17, 10] = 4 * waypoints[i + 1].time ** 3
            A[17, 11] = 3 * waypoints[i + 1].time ** 2
            A[17, 12] = 2 * waypoints[i + 1].time
            A[17, 13] = 1

        # =============================
        # Equality vector (right side)
        # =============================
        b = zeros((numConstraints, 1))

        b[0, 0] = waypoints[i].x
        b[1, 0] = waypoints[i].y
        b[2, 0] = waypoints[i].z
        b[3, 0] = waypoints[i].yaw
        b[4, 0] = waypoints[i+1].x
        b[5, 0] = waypoints[i+1].y
        b[6, 0] = waypoints[i+1].z
        b[7, 0] = waypoints[i+1].yaw

        # Derivatives = 0 for absolute Start, else Rendezvous of Segments
        if i != 0:
            b[8, 0] = 4 * trajectory[-1][0] * waypoints[i].time ** 3 + 3 * trajectory[-1][1] * waypoints[i].time ** 2 + 2 * trajectory[-1][2] * waypoints[i].time + trajectory[-1][3]
            b[9, 0] = 4 * trajectory[-1][5] * waypoints[i].time ** 3 + 3 * trajectory[-1][6] * waypoints[i].time ** 2 + 2* trajectory[-1][7] * waypoints[i].time + trajectory[-1][8]
            b[10, 0] = 4 * trajectory[-1][10] * waypoints[i].time ** 3 + 3 * trajectory[-1][11] * waypoints[i].time ** 2 + 2 * trajectory[-1][12] * waypoints[i].time + trajectory[-1][13]
            b[11, 0] = 2 * trajectory[-1][15] * waypoints[i].time + trajectory[-1][16]

            b[12, 0] = 12 * trajectory[-1][0] * waypoints[i].time ** 2 + 6 * trajectory[-1][1] * waypoints[i].time + 2 * trajectory[-1][2]
            b[13, 0] = 12 * trajectory[-1][5] * waypoints[i].time ** 2 + 6 * trajectory[-1][6] * waypoints[i].time + 2 * trajectory[-1][7]
            b[14, 0] = 12 * trajectory[-1][10] * waypoints[i].time ** 2 + 6 * trajectory[-1][11] * waypoints[i].time + 2 * trajectory[-1][12]


        # =============================
        # Solver Setup
        # =============================
        # OSQP needs:
        # P = quadratic terms
        # q = linear terms
        # A = constraint matrix of ALL constraints (inequality & equality)
        # l = lower constraints
        # u = upper constraints
        P = csc_matrix(P)
        q = hstack(q)
        h = hstack(h)
        b = hstack(b)

        A = vstack([G, A])
        A = csc_matrix(A)
        l = -inf * ones(len(h))
        l = hstack([l, b])
        u = hstack([h, b])

        # setup solver and solve
        m = osqp.OSQP()
        m.setup(P=P, q=q, A=A, l=l, u=u) # extra solver variables can be set here
        res = m.solve()

        # save to trajectory variable
        trajectory.append(res.x)
        print("QP solution Number ", i, "following: ", res.x)

    return trajectory




def planner(waypoint_arr, isJoint):
    # test waypoints
    #waypoint_arr = []
    #waypoint_arr.append([0, 0, 0, 2, 0])
    #waypoint_arr.append([1, 5, 0, 4, 3])
    #waypoint_arr.append([2, 5, 5, 3, 1])
    #waypoint_arr.append([3, 0, 5, 1, 5])
    #waypoint_arr.append([4, -5, 0, 2, 4])

    waypoints = []


    # the given "waypoints" are just the x,y,z,yaw values -> convert them to actual Waypoints
    for i in range(size(waypoint_arr)):
        # calculate time of waypoint
        if i == 0:
            time = 0
        else:
            time += calc_time(waypoint_arr[i-1], waypoint_arr[i])
        # create and append waypoint
        waypoint = Waypoint(waypoint_arr[i][1], waypoint_arr[i][2], waypoint_arr[i][3], waypoint_arr[i][4], time)
        waypoints.append(waypoint)


    if isJoint:
        trajectory = joint(waypoints)
    else:
        trajectory = separate(waypoints)

    # show generated trajectory in new window
    draw.draw_traj(waypoints, trajectory)

    # after closing trajectory visualization
    return waypoints,trajectory
