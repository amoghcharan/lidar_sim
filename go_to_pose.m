% Initializing the agents to random positions with barrier certificates.
% This script shows how to initialize robots to a particular pose.
% Paul Glotfelter edited by Sean Wilson
% Modified for LiDAR function integration
% 07/2019

N = 3;
initial_positions = generate_initial_conditions(N, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

% Set some parameters for use with the barrier certificates.  We don't want
% our agents to collide

% Create a barrier certificate for use with the above parameters
unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();

%Get randomized initial conditions in the robotarium arena
final_goal_points = generate_initial_conditions(N, ...
    'Width', r.boundaries(2)-r.boundaries(1)-r.robot_diameter, ...
    'Height', r.boundaries(4)-r.boundaries(3)-r.robot_diameter, ...
    'Spacing', 0.5);

args = {'PositionError', 0.025, 'RotationError', 0.05};
init_checker = create_is_initialized(args{:});
controller = create_waypoint_controller(args{:});

% Get initial location data for while loop condition.
x=r.get_poses();
r.step();

% Sample Rectangle
% lx = -1;
% rx = -0.3;
% ly = -0.2;
% ry = 0.5;
% r2 = rectangle('Position', [lx,ly, abs(rx-lx), abs(ry-ly)],'EdgeColor', 'r',...
%                'LineWidth', 2);

% Define Obstacles by sequential vertice coordinates
r1 = [-1, -1];
r2 = [-1, 0];
r3 = [0, -1];

l1 = [1.1, 0.8];
l2 = [0.6, 0.8];
l3 = [0.6, 0.5];
l4 = [0.5, 0.3];

% Plotting Setup for shapes
line_width = 3;

lr1 = line([r1(1), r2(1)], [r1(2), r2(2)], 'LineWidth', line_width, 'Color', 'k')
lr2 = line([r2(1), r3(1)], [r2(2), r3(2)], 'LineWidth', line_width, 'Color', 'k')
lr3 = line([r1(1), r3(1)], [r1(2), r3(2)], 'LineWidth', line_width, 'Color', 'k')

ll1 = line([l1(1), l2(1)], [l1(2), l2(2)], 'LineWidth', line_width, 'Color', 'k')
ll2 = line([l2(1), l3(1)], [l2(2), l3(2)], 'LineWidth', line_width, 'Color', 'k')
ll3 = line([l3(1), l4(1)], [l3(2), l4(2)], 'LineWidth', line_width, 'Color', 'k')
ll4 = line([l1(1), l4(1)], [l1(2), l4(2)], 'LineWidth', line_width, 'Color', 'k')

% Structure vertices as cell array
obs = {
    {[r1; r2; r3]};
    {[l1; l2; l3; l4]}
    };

% Plotting Initialization
% Choose color of points in RGB values
RGB = [0 190 119]/256;
scan = [0,0];
h = scatter(scan(:,1), scan(:,2), 144, RGB, 'filled');

while(~init_checker(x, final_goal_points))
    x = r.get_poses();
    dxu = controller(x, final_goal_points);
    dxu = unicycle_barrier_certificate(dxu, x);

    % Call LIDAR to find all plottable intersection points
    scan = LIDAR(x, N, 18, 0.5, obs, 'Gaussian');

    % Plot intersection points
    [numP,numCols] = size(scan);

    for g = 1:numP
        % Update LiDAR scatter
        set(h,'XData',scan(:,1),'YData',scan(:,2));
    end

    r.set_velocities(1:N, dxu);
    r.step();
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
