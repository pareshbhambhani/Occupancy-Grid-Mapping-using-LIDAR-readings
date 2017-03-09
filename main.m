clear all;
close all;

load practice.mat 
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
%     You may not use time info for implementation.
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement (in meter) at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles (in radian) the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] pose is 3-by-K array containing the pose of the mobile robot over time. 
%     e.g. pose(:,k) is the [x(meter),y(meter),theta(in radian)] at time index k.

% 1. Decide map resolution, i.e., the number of grids for 1 meter.
param.resol = 25;

% 2. Decide the initial map size in pixels
param.size = [900, 900];

% 3. Indicate where you will put the origin in pixels
param.origin = [700,600]'; 

% 4. Log-odd parameters 
param.lo_occ = 1;
param.lo_free = 0.5; 
param.lo_max = 100;
param.lo_min = -100;


% 5. Recod video Bool
param.captureBool = 0;

% Mapping function
myMap = occGridMapping(ranges, scanAngles, pose, param);

% The final grid map: 
figure(2),
imagesc(myMap);
colormap('gray'); axis equal;
