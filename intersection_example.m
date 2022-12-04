% Example of static point and line-line intersection with obstacle edges
% Amogh Chinnakonda

clear
clc

% Inputs Needed
x1 = -3;
y1 = -3;
x2 = 4;
y2 = -6;
lx = -2;
rx = -5;
ly = -2;
ry = -5;

v1 = [-2,-2];
v2 = [-5,-2];
v3 = [-2,-5];
v4 = [-5,-5];

v5 = [-1,-1];
v6 = [-1, 5.5];
v7 = [3.5, -1];
v8 = [3.5, 5.5];

% Static Variables
cpos = [x1,y1];

rect = [v1; v2; v3; v4];
r1 = [v1, v2 ; v1, v3; v2, v4; v3, v4; v5, v6; v5, v7; v6, v8; v7, v8]
[numRows,numCols] = size(r1);
intpoints = []
midpoints = []
norms = []

% Dynamic Variables
lpos = [x2,y2];
a = [cpos(1), cpos(2); lpos(1), lpos(2)];

    for k = 1:numRows

        v = r1(k,1:4);
        x3 = v(1)
        y3 = v(2)
        x4 = v(3)
        y4 = v(4)

        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
        u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))

        px1 = x1 + t*(x2-x1);
        py1 = y1 + t*(y2-y1);
        px2 = x3 + u*(x4-x3);
        py2 = y3 + u*(y4-y3);

        if (t>=0) && (t<=1) && (u>=0) && (u<=1)
            intpoints = [intpoints; [px2,py2]]
        else
            intpoints = [intpoints]
        end

    end

    [numR,numCol] = size(intpoints);

    for j = 1:numR
        norms = [norms, norm([intpoints(j,1)-x1, intpoints(j,2)-y1])]
    end

    [M,I] = min(norms)
    midpoints = [intpoints(I,:)]
    [numM,numC] = size(midpoints);

    hold on
    axis = [0 10 0 12];
    l1 = line([cpos(1), lpos(1)], [cpos(2), lpos(2)])
    r2 = rectangle('Position', [rx,ry, abs(rx-lx), abs(ry-ly)])
    r3 = rectangle('Position', [v5(1),v5(2), v8(1)-v5(1), v8(2)-v5(2)])
    RGB = [0 155 119]/256;
    scatter(midpoints(1), midpoints(2),[144], RGB, 'filled')
