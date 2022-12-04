% LIDAR Function to generate points around robot with angle resolution and
% lidar range and importable obstacles
% Amogh Chinnakonda

function Lidar(x1, y1, ang_Res, dist, rect)
% ang_Res = the resolution of the simulated LiDAR
% dist = the projected maximum distance of the LiDAR's range
% rect = an importable rectangle into the figure

r = 0:ang_Res:360;
x2 = [];
y2 = [];
cpos = [x1,y1];

% The edges of the figure
v1 = [-1.6,-1];
v2 = [-1.6, 1];
v3 = [1.6, -1];
v4 = [1.6, 1];

% List of intersection points
new_int = []

% Creating LiDAR Range Vectors around Robot's current position
for i = 1:length(r)
    x2(i) = x1+dist*cosd(r(i));
    y2(i) = y1+dist*sind(r(i));

    lpos = [x2(i),y2(i)];
    a = [x1, y1; x2(i), y2(i)];
    r1 = [v1, v2 ; v1, v3; v2, v4; v3, v4; rect(1,1:2), rect(2,1:2); rect(1,1:2), rect(3,1:2); rect(4,1:2), rect(3,1:2); rect(4,1:2), rect(2,1:2)];
    [numRows,numCols] = size(r1);
    intpoints = []
    norms = []
    midpoints = []

    % Using line-line intersection to define potential intersecting points between robot position and obstacle edges
    for k = 1:numRows

        v = r1(k,1:4);
        x3 = v(1)
        y3 = v(2)
        x4 = v(3)
        y4 = v(4)

        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4))/((x1-x2(i))*(y3-y4) - (y1-y2(i))*(x3-x4))
        u = ((x1-x3)*(y1-y2(i)) - (y1-y3)*(x1-x2(i)))/((x1-x2(i))*(y3-y4) - (y1-y2(i))*(x3-x4))

        px = cpos(1) + t*(lpos(1) - cpos(1))
        py = cpos(2) + t*(lpos(2) - cpos(2))

        if (t>=0) && (t<=1) && (u>=0) && (u<=1)
            midpoints = [midpoints; [px,py]]
        else
            midpoints = [midpoints]
        end

        % Find the intersections that are the least distance away on the same line
        [numM,numC] = size(midpoints);
        for j = 1:numM
            norms = [norm([midpoints(j,1)-x1, midpoints(j,2)-y1])]
        end

        [M,I] = min(norms)
        intpoints = [midpoints(I,:)]

    end

    % Display a scatter of all intersected points on figure
    [numR,numCols] = size(intpoints);
    for g = 1:numR
        RGB = [0 155 119]/256;

        h = scatter(intpoints(g,1), intpoints(g,2),[144], RGB, 'filled')

    end
end

