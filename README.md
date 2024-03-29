# LiDAR Sim

#### :fireworks: A 2D LiDAR visualization for simulated robots in MATLAB at the Georgia Tech Robotarium

#### :wavy_dash: Ability to modify distance, angular resolution, and Gaussian distortion of points

#### :mount_fuji: Works with any N-vertice shape and multiple robots simultaneously

Usage:

`LIDAR(x, N, ang_Res, dist, obs, error)`

- `x` is the current robot's x,y position
- `N` is the number of active robots
- `ang_Res` is how many 360/N rays to be generated
- `dist` is the maximum distance range of a ray
- `obs` are predefined obstacles given by its vertices
- `error` is defined as either 'None' or 'Gaussian'

returns
- `output`: a matrix of all calculated intersection points [x,y]

Refer to the `go_to_pose.m` file for an example of the function call, obstacle initialization, and plotting routine

#
![Example of LiDAR function with Robotarium robots](images/lidar_mov_2.gif "Example of LiDAR function with Robotarium robots")

#

Plotting: 

- Initialize Plot of LIDAR points before experiment loop:
```
RGB = [255 0 0]/256;  
scan = [0,0];  
h = scatter(scan(:,1), scan(:,2), 72, RGB, 'filled'); 
```
- In loop plotting update:
```
[numP,numCols] = size(scan);     
for g = 1:numP
    set(h,'XData',scan(:,1),'YData',scan(:,2));      
end
```


#
Warning:
* The real-time run speed of this function is variable with the amount of intersections that are computed and plotted, thus simpler (less robots, less obstacles) experiments are more likely to run 
* As `N` number of robots increase, `ang_res` and `dist` need to be lowered to ensure real-time operation on Robotarium
* See the following table for sample runs and respective real-time run speed

### Point Cloud Test:
| N | ang_Res | dist | Time w/ Plotting (s) | Time w/o Plotting (s) |
| -- | -- | -- | -- | -- |
| 1 | 1 | 0.5 | 0.0295 | 0.0331|
| 2 | 1 | 0.5 | 0.0414 | 0.0347 |
| 3 | 1 | 0.5 | 0.1094 | 0.0829 |
| 1 | 5 | 0.5 | 0.0138 | 0.0326 |
| 2 | 5 | 0.5 | 0.0291 | 0.0278 |
| 3 | 5 | 0.5 | 0.0350 | 0.0233 |
| 5 | 5 | 0.5 | 0.0428 | 0.0450 |
| 6 | 18 | 0.5 | 0.0332 | 0.0331 |
| 8 | 18 | 0.5 | 0.0513 | 0.0452 |
| 10 | 36 | 0.5 | 0.0402 | 0.0353 |



