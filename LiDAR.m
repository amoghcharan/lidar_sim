% LIDAR Function to generate points around robot with angle resolution and
% lidar range and importable obstacles
% Amogh Chinnakonda

function intpoints = LIDAR(x, N, ang_Res, dist, obs)

% First point generated is at 0 degrees to the right
% x1 is the current robot's x position
% y1 is the current robot's y position
% ang_Res is how many 360/N rays to be generated
% dist is the maximum distance range of a ray
% rect is a rectangular obstacle given by its vertices

% Number of LiDAR rays
r = 0:ang_Res:360;

% Outer Vertices of Robotarium
v1 = [-1.6,-1];
v2 = [-1.6, 1];
v3 = [1.6, -1];
v4 = [1.6, 1];

% Empty array of shortest intersection vectors
intpoints = [];

% Add dynamic rectangles based on each robot position
% currrect = {};
%
% for c = 1:N
%     xc = x(1,c)
%     yc = x(2,c)
%     ac = x(3,c)
%     theta = [
%         ((pi/4) + ac);
%         ((3*pi/4) + ac);
%         ((5*pi/4) + ac);
%         ((7*pi/4) + ac);
%         ]
%     for t = 1:4
%         if theta(t) > 2*pi
%             theta(t) = theta(t) - (2*pi)
%         end
%     end
%     vc1 = [xc + cos(theta(1)), yc + sin(theta(1))];
%     vc2 = [xc + cos(theta(2)), yc + sin(theta(2))];
%     vc3 = [xc + cos(theta(3)), yc + sin(theta(3))];
%     vc4 = [xc + cos(theta(4)), yc + sin(theta(4))];
%     currrect = {currrect; [vc1, vc2]; [vc2, vc3]; [vc3, vc4]; [vc4, vc1]}
%
% end

% Define x,y of each robot
for h = 1:N
    x1 = x(1,h);
    y1 = x(2,h);

    % All endpoints of rays at current position
    x2 = [];
    y2 = [];
    cpos = [x1,y1];

    % Define size of imported obstacles
    sz = size(obs);

    % Main Ray Generating Algorithm
    for i = 1:length(r)

        % Current projected end position of ray
        x2(i) = x1+dist*cosd(r(i));
        y2(i) = y1+dist*sind(r(i));
        lpos = [x2(i),y2(i)];

        % All obstacle lines
        vcomb = [v1, v2 ; v1, v3; v2, v4; v3, v4]; %rect(1,1:2), rect(2,1:2); rect(1,1:2), rect(3,1:2); rect(4,1:2), rect(3,1:2); rect(4,1:2), rect(2,1:2)];

        for m = 1:sz(1)

            % Checking vertices of each obstacle
            e = obs{m,:}{1,:};
            sze = size(e);
            numv = sze(1);

            % Creating first and last connection line segment
            vcomb = [vcomb; e(1,:) e(end,:)];

            % Adding inner vertice connection line segments
            for n = 1:(numv-1)
                vcomb = [vcomb; e(n,:) e(n+1,:)];
            end
        end

        [numLines,numCols] = size(vcomb);

        % All intersection points on obstacle lines
        midpoints = [];

        % Magnitude of intersection vectors
        norms = [];

        for k = 1:numLines

            % Current positions of line vertices
            v = vcomb(k,1:4);
            x3 = v(1);
            y3 = v(2);
            x4 = v(3);
            y4 = v(4);

            % Line-Line Intersection Parameter Calculations
            t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4))/((x1-x2(i))*(y3-y4) - (y1-y2(i))*(x3-x4));
            u = ((x1-x3)*(y1-y2(i)) - (y1-y3)*(x1-x2(i)))/((x1-x2(i))*(y3-y4) - (y1-y2(i))*(x3-x4));

            % Potential intersection point
            px = cpos(1) + t*(lpos(1) - cpos(1));
            py = cpos(2) + t*(lpos(2) - cpos(2));

            % When both parameters are between 0 & 1, intersection is found
            if (t>=0) && (t<=1) && (u>=0) && (u<=1)
                midpoints = [midpoints; [px,py]];
            else
                midpoints = [midpoints];
            end

            % Populate Norms with magnitudes of potential endpoints
            [numR,numC] = size(midpoints);
            for j = 1:numR
                norms(j) = [norm([midpoints(j,1)-x1, midpoints(j,2)-y1])];
            end

            % Find closest intersections in [intpoints]
            [M,I] = min(norms);
            intpoint = midpoints(I,:);

        end

        % Append shortest intersection points
        tf = isempty(intpoint);
        if tf == 0
            % [int_x, int_y, int_dist]
            intpoints = [intpoints; [intpoint(1), intpoint(2), M]];
        else
            intpoints = [intpoints];
        end


        % Add Gausian Distortion to points
    %     for l = 1:length(intpoints(:,1))
    %         intang = [

    end

end


