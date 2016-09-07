function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y
    %% Check if the first time running this function
    if previous_t < 0
        state = [x, y, 0, 0];
        param.P = 2 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end
    %% Place parameters like covarainces, etc. here:
    dt = t - previous_t;
    sigm = [dt*dt/4 0 dt/2 0; 
            0 dt*dt/4 0 dt/2; 
            dt/2 0 1 0; 
            0 dt/2 0 1];
    sigo = 1e-2 * eye(2);
    A = [1 0 dt 0; 
        0 1 0 dt; 
        0 0 1 0; 
        0 0 0 1];
    C = [1 0 0 0; 
        0 1 0 0];  
    %% TODO: Add Kalman filter updates
    param.R = sigo;
    P = A * param.P * A' + sigm;
    K = P*C' * (param.R + C*P*C')^-1;
    state = A * state' + K * ([x,y]' - C * A * state');
    state = state';
    param.P = P - K * C * P;
    predictx = state(1) + (state(3) * dt * 10);
    predicty = state(2) + (state(4) * dt * 10);
    return;
end
