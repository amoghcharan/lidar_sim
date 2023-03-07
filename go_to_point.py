import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import time

# Instantiate Robotarium object
N = 1
initial_conditions = np.array(np.mat('1 0.5 -0.5 0 0.28; 0.8 -0.3 -0.75 0.1 0.34; 0 0 0 0 0'))

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=generate_initial_conditions(N), sim_in_real_time=False)

# Define goal points by removing orientation from poses
goal_points = generate_initial_conditions(N)

# Create single integrator position controller
single_integrator_position_controller = create_si_position_controller()

# Create barrier certificates to avoid collision
#si_barrier_cert = create_single_integrator_barrier_certificate()
si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

_, uni_to_si_states = create_si_to_uni_mapping()

# Create mapping from single integrator velocity commands to unicycle velocity commands
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

# define x initially
x = r.get_poses()


# Define Obstacles by sequential vertex coordinates
r1 = [-1, -1]
r2 = [-1, 0]
r3 = [0, -1]

l1 = [1.1, 0.8]
l2 = [0.6, 0.8]
l3 = [0.6, 0.5]
l4 = [0.5, 0.3]

xr12 = [r1[0], r2[0]]
yr12 = [r1[1], r2[1]]
xr13 = [r1[0], r3[0]]
yr13 = [r1[1], r3[1]]
xr32 = [r3[0], r2[0]]
yr32 = [r3[1], r2[1]]

xl12 = [l1[0], l2[0]]
yl12 = [l1[1], l2[1]]
xl23 = [l3[0], l2[0]]
yl23 = [l3[1], l2[1]]
xl34 = [l3[0], l4[0]]
yl34 = [l3[1], l4[1]]
xl14 = [l1[0], l4[0]]
yl14 = [l1[1], l4[1]]

plt.plot(xr12, yr12, 'k', linestyle="--")
plt.plot(xr13, yr13, 'k', linestyle="--")
plt.plot(xr32, yr32, 'k', linestyle="--")
plt.plot(xl12, yl12, 'k', linestyle="--")
plt.plot(xl23, yl23, 'k', linestyle="--")
plt.plot(xl34, yl34, 'k', linestyle="--")
plt.plot(xl14, yl14, 'k', linestyle="--")
robot_markers = r.axes.scatter([],[])

obs = [
    [r1, r2, r3],
    [l1, l2, l3, l4]
]
# ads = np.array([[obs[0][0][0], obs[0][0][1], obs[0][-1][0], obs[0][-1][1]]])

# print(f"Obstacles 0 :{obs[0]}")
# print(f"Obstacles 1 :{obs[1]}")
# print(f"Obstacles Shape :{ads.shape}")

x_si = uni_to_si_states(x)
r.step()

# While the number of robots at the required poses is less
# than N...
while (np.size(at_pose(np.vstack((x_si,x[2,:])), goal_points, rotation_error=100)) != N):

    # Get poses of agents
    x = r.get_poses()

    # Inputs
    ang_Res = 18
    dist = 0.2

    # Algorithm test
    # Mimic ang_Res input of MATLAB
    res = (360.0 / ang_Res) + 1.0
    ra = np.linspace(0, 360, num=int(res))

    # Outer Vertices of Robotarium
    v1 = np.array([-1.6, -1.0])
    v2 = np.array([-1.6, 1.0])
    v3 = np.array([1.6, -1.0])
    v4 = np.array([1.6, 1.0])

    vvert = [
        [v1[0], v1[1], v2[0], v2[1]],
        [v1[0], v1[1], v3[0], v3[1]],
        [v2[0], v2[1], v4[0], v4[1]],
        [v3[0], v3[1], v4[0], v4[1]],
    ]

    vcomb = np.array(vvert)

    for v in range(len(obs)):
        vcomb = np.append(vcomb, np.array([[obs[v][0][0], obs[v][0][1], obs[v][-1][0], obs[v][-1][1]]]), axis=0)
        for w in range(len(obs[v])-1):
            vcomb = np.append(vcomb, np.array([[obs[v][w][0], obs[v][w][1], obs[v][w+1][0], obs[v][w+1][1]]]), axis=0)


    # Empty array of shortest intersection vectors
    intpoints = np.empty((0,5), float)
    gauspoints = np.empty((0,2), float)
    curr_rect = np.empty((0, 4), float)

    # Add dynamic rectangles based on each robot position
    for c in range(N):
        xc = x[0, c]
        yc = x[1, c]
        ac = x[2, c]
        theta = [
        ((np.pi / 4) + ac),
        ((3 * np.pi / 4) + ac),
        ((5 * np.pi / 4) + ac),
        ((7 * np.pi / 4) + ac)
        ]
        for t in range(4):
            if theta[t] > 2 * np.pi:
                theta[t] = theta[t] - (2 * np.pi)
        rd = 0.15
        vc1 = [xc + rd * np.cos(theta[0]), yc + rd * np.sin(theta[0])]
        vc2 = [xc + rd * np.cos(theta[1]), yc + rd * np.sin(theta[1])]
        vc3 = [xc + rd * np.cos(theta[2]), yc + rd * np.sin(theta[2])]
        vc4 = [xc + rd * np.cos(theta[3]), yc + rd * np.sin(theta[3])]
        new_rect = [
            [vc1[0], vc1[1], vc2[0], vc2[1]],
            [vc1[0], vc1[1], vc3[0], vc3[1]],
            [vc2[0], vc2[1], vc4[0], vc4[1]],
            [vc3[0], vc3[1], vc4[0], vc4[1]],
        ]
        curr_rect = np.append(curr_rect, new_rect, axis=0)

    # print(f"New_Rect: {new_rect}")
    # print(f"Curr_Rect Shape:{curr_rect.shape}")


    # Define x,y of each robot
    for h in range(N):
        x1 = x[0, h]
        y1 = x[1, h]
        z1 = x[2, h]

        # All endpoints of rays at current position
        x2 = np.array([])
        y2 = np.array([])
        cpos = np.array([x1, y1])
        # print(cpos)

        # Define size of imported obstacles

        # INSERT CODE HERE

        # Main Ray Generating Algorithm
        # print(f"Current Robot is: {h}")

        # All intersection points on obstacle lines
        midpoints = np.empty((0,2), float)

        # All obstacles lines

        # Find all other robot's boundaries
        # print(f"Before Delete: {curr_rect}")
        vdy = np.delete(curr_rect, [(h*4), (h*4)+1, (h*4)+2, (h*4)+3], axis=0)
        # print(f"After: {vdy}")
        vcomb = np.append(vcomb, vdy, axis=0)
        # print(f"VComb: {vcomb}")



        # Current projected end position of ray
        for i in range(len(ra)):
            x2 = np.append(x2, x1 + dist * np.cos(np.deg2rad(ra[i])))
            y2 = np.append(y2, y1 + dist * np.sin(np.deg2rad(ra[i])))
            lpos = np.array([x2[i], y2[i]])

            # Shape of final vertices matrix
            numLines, numCols = vcomb.shape



            # # Magnitude of intersection vectors
            # norms = np.empty((0,1), float)

            for k in range(numLines):
                # Current positions of line vertices
                v = vcomb[k]
                x3 = v[0]
                y3 = v[1]
                x4 = v[2]
                y4 = v[3]

                if x1 != x2[i] and y1 != y2[i]:
                    # Line-Line Intersection Parameter Calculations
                    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4))/((x1-x2[i])*(y3-y4) - (y1-y2[i])*(x3-x4))
                    u = ((x1-x3)*(y1-y2[i]) - (y1-y3)*(x1-x2[i]))/((x1-x2[i])*(y3-y4) - (y1-y2[i])*(x3-x4))

                    # Potential Intersection Point
                    px = cpos[0] + t*(lpos[0] - cpos[0])
                    py = cpos[1] + t*(lpos[1] - cpos[1])

                # When both parameters are between 0 & 1, intersection is found
                if t>=0 and t<=1 and u>=0 and u<=1:
                    new_point = np.array([[float(px),float(py)]])
                    midpoints = np.append(midpoints, new_point, axis=0)

                # Magnitude of intersection vectors
                norms = np.empty((0, 1), float)

                if midpoints.size > 0 :
                    numR, numC = midpoints.shape
                    for j in range(numR):
                        new_norm = np.linalg.norm(
                           [midpoints[j,0]-x1, midpoints[j,1]-y1]
                        )
                        norms = np.append(norms, np.array([[new_norm]]), axis=0)

            # Find closest intersections in [intpoints]
            if norms.size > 0:
                min_ndx = np.argmin(norms)
                M = np.min(norms)
                new_int = np.array([[midpoints[min_ndx,0], midpoints[min_ndx,1], M, i, h]])
                # print(f"New int: {new_int.shape}")

                intpoints = np.append(intpoints, new_int, axis=0)
                # print(f"Intpoints: {intpoints}")

            # Add Gausian Distortion to points
            zbounds = [0, 0.5]
            z = np.random.uniform(zbounds[0], zbounds[1], 1)
            sigma = 0.05
            if intpoints.size > 0:
                for p in range(len(intpoints[:, 1])):
                    dist_g = intpoints[p, 2] + z * sigma
                    # print(f"Gauss Dist = {dist_g}")
                    # print(f"X Gauss = {intpoints[p, 4]}")
                    gauspointsx = x[0, int(intpoints[p, 4])] + dist_g * np.cos(np.deg2rad(int(intpoints[p, 3])))
                    gauspointsy = x[1, int(intpoints[p, 4])] + dist_g * np.sin(np.deg2rad(int(intpoints[p, 3])))
                    gaus_arr = np.array([[gauspointsx[0], gauspointsy[0]]])
                    gauspoints = np.append(gauspoints, gaus_arr, axis=0)
                    # print(f"GausPoints: {gauspoints}")
                    # print(f"Array: {gaus_arr.shape}")
                    # gauspoints[p, 1] = x[1, (intpoints[p, 4])] + dist_g * np.sind(r(intpoints(p, 4)))



    def update(i):
        robot_markers.set_offsets(intpoints)

    anim = FuncAnimation(r.figure, update)

    x_si = uni_to_si_states(x)

    # Create single-integrator control inputs
    dxi = single_integrator_position_controller(x_si, goal_points[:2][:])

    # Create safe control inputs (i.e., no collisions)
    dxi = si_barrier_cert(dxi, x_si)

    # Transform single integrator velocity commands to unicycle
    dxu = si_to_uni_dyn(dxi, x)

    # Set the velocities by mapping the single-integrator inputs to unciycle inputs
    r.set_velocities(np.arange(N), dxu)
    # Iterate the simulation
    r.step()

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
