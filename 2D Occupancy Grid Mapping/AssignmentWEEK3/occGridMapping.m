% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)
    %% Parameters
    % the number of grids for 1 meter.
    myResol = param.resol;
    % the initial map size in pixels
    myMap = zeros(param.size);
    % the origin of the map in pixels
    myOrigin = param.origin; 
    % Log-odd parameters 
    lo_occ = param.lo_occ;
    lo_free = param.lo_free; 
    lo_max = param.lo_max;
    lo_min = param.lo_min;
    %% Loop
    N = size(pose,2);
    K = size(ranges, 1);
    for i = 1:N % for each time
        pose_orig = [pose(1, i), pose(2, i)];
        theta = pose(3, i);
        % local coordinates of occupied points in real world
        occ_local = [ranges(:,i) .* cos(theta + scanAngles), -ranges(:,i) .* sin(theta + scanAngles)];
        % coordinate of robot in metric map
        pose_grid = ceil(myResol * pose_orig);
        for j = 1:K
            % Find grids hit by the rays (in the gird map coordinate)
            occ_grid = ceil(myResol * (occ_local(j,:) + pose_orig));
            % Find occupied-measurement cells and free-measurement cells
            [freex, freey] = bresenham(pose_grid(1), pose_grid(2), occ_grid(1), occ_grid(2));
            free = sub2ind(size(myMap),freey + myOrigin(2), freex + myOrigin(1));
            occ = sub2ind(size(myMap), occ_grid(2) + myOrigin(2), occ_grid(1) + myOrigin(1));
            % Update the log-odds
            myMap(free) = myMap(free) - lo_free;
            myMap(occ) = myMap(occ) + lo_occ;
        end
    end
    myMap(myMap < lo_min) = lo_min;
    myMap(myMap > lo_max) = lo_max;
end

