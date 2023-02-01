import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import matplotlib.pyplot as plt
import numpy as np
import time

# Instantiate Robotarium object
N = 5
initial_conditions = np.array(np.mat('1 0.5 -0.5 0 0.28; 0.8 -0.3 -0.75 0.1 0.34; 0 0 0 0 0'))

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

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
x_si = uni_to_si_states(x)
r.step()

# While the number of robots at the required poses is less
# than N...
while (np.size(at_pose(np.vstack((x_si,x[2,:])), goal_points, rotation_error=100)) != N):

    # Get poses of agents
    x = r.get_poses()

    # Inputs
    ang_Res = 36
    dist = 0.5

    # Algorithm test
    # Mimic ang_Res input of MATLAB
    res = (360.0 / ang_Res) + 1.0
    ra = np.linspace(0, 360, num=int(res))

    # Outer Vertices of Robotarium
    v1 = np.array([-1.6, -1])
    v2 = np.array([-1.6, 1])
    v3 = np.array([1.6, -1])
    v4 = np.array([1.6, 1])

    vvert = [
        [v1[0], v1[1], v2[0], v2[1]],
        [v1[0], v1[1], v3[0], v3[1]],
        [v2[0], v2[1], v4[0], v4[1]],
        [v3[0], v3[1], v4[0], v4[1]],
    ]

    # Empty array of shortest intersection vectors
    intpoints = []

    # Add dynamic rectangles based on each robot position

    # INSERT CODE HERE

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
        print(f"Current Robot is: {h}")

        # Current projected end position of ray
        for i in range(len(ra)):
            x2 = np.append(x2, x1 + dist * np.cos(np.deg2rad(ra[i])))
            y2 = np.append(y2, y1 + dist * np.sin(np.deg2rad(ra[i])))
            lpos = np.array([x2[i], y2[i]])

            # All obstacles lines
            vcomb = np.array(vvert)

            # Find all other robot's boundaries

            # INSERT CODE HERE



            # Shape of final vertices matrix
            numLines, numCols = vcomb.shape

            # All intersection points on obstacle lines
            midpoints = np.array([])

            # Magnitude of intersection vectors
            norms = np.array([])

            for k in range(numLines):
                # Current positions of line vertices
                v = vcomb[k]
                x3 = v[0]
                y3 = v[1]
                x4 = v[2]
                y4 = v[3]

                # Line-Line Intersection Parameter Calculations
                t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4))/((x1-x2[i])*(y3-y4) - (y1-y2[i])*(x3-x4))
                u = ((x1-x3)*(y1-y2[i]) - (y1-y3)*(x1-x2[i]))/((x1-x2[i])*(y3-y4) - (y1-y2[i])*(x3-x4))

                # Potential Intersection Point
                px = cpos[0] + t*(lpos[0] - cpos[0])
                py = cpos[1] + t*(lpos[1] - cpos[1])


                # When both parameters are between 0 & 1, intersection is found
                if t>=0 and t<=1 and u>=0 and u<=1:
                    # print(px, py)
                    midpoints = np.append(midpoints, [px,py], axis=0)


            if midpoints.size>0 :
                print(midpoints.shape)
                numR, numC = midpoints.shape
                for j in range(numR):
                    norms[j] = np.linalg.norm(
                       [midpoints[j,0]-x1, midpoints[j,1]-y1]
                    )
                # print(midpoints)
                print(np.argmin(norms))
                # Find closest intersections in [intpoints]
                # intpoints = np.append()

                # Populate Norms with magnitudes of potential endpoints
                # if midpoints:
                #     print(midpoints.shape)
                #     numR, numC = midpoints.shape
                #     for j in range(numR):
                #         norms[j] = np.linalg.norm(
                #            [midpoints[j,0]-x1, midpoints[j,1]-y1]
                #         )
                #     # print(midpoints)
                #     print(np.argmin(norms))
                # Find closest intersections in [intpoints]
                # intpoints = np.append()






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
    plt.show

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
