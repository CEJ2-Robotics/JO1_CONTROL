% robocar with U-turn tracking
clear all; close all; clc;
Simulink.importExternalCTypes("types.h")

% Vehicle parameters
params.L = 0.5;        % distance between front and rear axle [m]
params.d = 0.4;        % Distance between the rear wheels (m)
params.R_w = 0.1;      % Radius of wheel (m)
params.delta_max = 0.5; % Maximum steering angle (rad)
params.v = 10;          % Forward velocity (m/s)

% Simulation parameters
dt = 0.001;      % time step [s]
t_final = 5;   % simulation duration [s]
t = 0:dt:t_final;

% Define U-turn track
% track.xd1 = [0.0 2.0 10.0 14.0 15.0];    % Coordinates of bottom half
% track.yd1 = [0 0 3.0 10.0 15.0];         % Coordinates of bottom half
% track.xd22 = [0 2.0 10.0 15.0];          % Coordinates of upper half
% track.yd22 = [30.0 30.0 27.0 20.0];      % Coordinates of upper half
% track.xd2 = fliplr(track.xd22);
% track.yd2 = fliplr(track.yd22);
% 
% track.x = [track.xd1, track.xd2];
% track.y = [track.yd1, track.yd2];

track.xd1 = [0.0 2.0 0.0];    % Coordinates of bottom half
track.yd1 = [0.0 2.0 4.0];         % Coordinates of bottom half

track.x = [track.xd1];
track.y = [track.yd1];

% Desired poles for controller
poles = [-20, -0.2];

% Design via pole placement
A_mat = [0, params.v; 0, 0];           % Linear A matrix
B_mat = [0; params.v/params.L];        % Linear B matrix
Kgains = place(A_mat, B_mat, poles);   % Find gains [k1 k2] to apply to x =[y psi]
params.k1 = Kgains(1);
params.k2 = Kgains(2);

% Initial conditions
x0 = 0;         % Initial x position, m
y0 = 0;         % Initial y position, m
theta0 = 0;     % Initial orientation, rad
state = [x0; y0; theta0];

% Arrays to store states and outputs for plotting
states = zeros(3, length(t));
states(:,1) = state;
deltaout = zeros(1, length(t));
erroryout = zeros(1, length(t));
errorpsiout = zeros(1, length(t));
desposout = zeros(2, length(t));
despsiout = zeros(1, length(t));

% Animation figure
figure('Name', 'Tricycle U-turn Tracking')
axis([min(track.x)-2 max(track.x)+2 min(track.y)-2 max(track.y)+2])
grid on
hold on

% Plot track
plot(track.x, track.y, 'r--', 'LineWidth', 2)
title('Tricycle U-turn Tracking')
xlabel('X Position [m]')
ylabel('Y Position [m]')

% Initialize track following variables
track_data.index = 1;
track_data.t_marker = 0;
track_data.A = [track.x(1); track.y(1)];
track_data.B = [track.x(2); track.y(2)];
track_data.base_norm = norm(track_data.B - track_data.A);
track_data.des_pos = [0; 0];

dpos_x = [];
dpos_y = [];
dphi = [];

% des_pos = [0, 0];
% des_psi = (3*pi)/2;

% Simulation loop
for i = 1:length(t)-1
    % Current state
    x = state(1);
    y = state(2);
    theta = state(3);
    
    % Update desired position on track
    [track_data, des_pos, des_psi] = update_track_position(track_data, track, t(i), params.v);
    
    % Calculate errors
    e_psi = theta - des_psi;
    e_y = y - des_pos(2);
    
    % Control law
    delta = calculate_steering(e_y, e_psi, params);
    
    % Store outputs
    deltaout(i) = delta;
    erroryout(i) = e_y;
    errorpsiout(i) = e_psi;
    desposout(:,i) = des_pos;
    despsiout(i) = des_psi;
    
    % Kinematic model
    state_dot = tricycle_kinematics(state, delta, params);
    
    % State update using euler
    state = state + dt * state_dot;
    states(:,i+1) = state;
    
    dpos_x = [dpos_x; des_pos(1)];
    dpos_y = [dpos_y; des_pos(2)];
    dphi = [dphi; des_psi];

    % Visualize tricycle
    if mod(i, 20) == 0  % smooth animation
        cla  % clear current axis
        plot(track.x, track.y, 'r--', 'LineWidth', 2)  % replot track
        visualize_tricycle(state(1), state(2), state(3), params)
        plot(states(1,1:i), states(2,1:i), 'b-')  % plot trajectory
        plot(des_pos(1), des_pos(2), 'g*', 'MarkerSize', 10)  % plot current target
        axis([min(track.x)-2 max(track.x)+2 min(track.y)-2 max(track.y)+2])
        grid on
        drawnow
        pause(0.01)
    end
end

% Final plots
plot_results(t, states, track, deltaout, erroryout, errorpsiout);

% Helper Functions
function state_dot = tricycle_kinematics(state, delta, params)
    % Compute state derivatives based on kinematic model with variable velocity
    v = params.v;
    x_dot = v * cos(state(3));
    y_dot = v * sin(state(3));

    disp(delta)

    theta_dot = (v/params.L) * tan(delta);
    state_dot = [x_dot; y_dot; theta_dot];
end

function delta = calculate_steering(e_y, e_psi, params)
    % Calculate steering angle with saturation
    delta = -params.k1 * e_y - params.k2 * e_psi;
    delta = max(min(delta, params.delta_max), -params.delta_max);
end

function [track_data, des_pos, des_psi] = update_track_position(track_data, track, t, v)
    % Check if we've reached the final point
    % if track_data.index >= length(track.x) - 1
    %     % Get the final point
    %     final_point = [track.x(end); track.y(end)];
    %     % Check if we're close enough to the final point (e.g., within 0.1 meters)
    %     if norm(track_data.des_pos - final_point) < 0.1
    %         % Set velocity to 0 to stop the vehicle
    %         v = 0;
    %     end
    % end

    % Update desired position along the track
    if norm(track_data.des_pos - track_data.A) >= track_data.base_norm
        track_data.index = min(track_data.index + 1, length(track.x) - 1);
        track_data.t_marker = t - (norm(track_data.des_pos - track_data.B)/v);
        track_data.A = [track.x(track_data.index); track.y(track_data.index)];
        track_data.B = [track.x(track_data.index + 1); track.y(track_data.index + 1)];
        track_data.base_norm = norm(track_data.B - track_data.A);
    end
    
    % if v == 0
    %     % If stopped, stay at current position
    %     track_data.des_pos = track_data.des_pos;
    % else
        % Normal update of desired position
        track_data.des_pos = track_data.A + v*(t - track_data.t_marker)* ...
                            (track_data.B - track_data.A)./norm(track_data.B - track_data.A);
    % end
    
    des_pos = track_data.des_pos;
    des_psi = atan2((track_data.B(2) - track_data.A(2)), (track_data.B(1) - track_data.A(1)));
end

function visualize_tricycle(x, y, theta, params)
    % Tricycle visualization parameters
    wheel_width = params.L/5;
    body_width = params.d;
    
    % Create transformation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Define vehicle body points
    body = [-params.L/2 -body_width/2;
            params.L/2  -body_width/2;
            params.L/2   body_width/2;
            -params.L/2  body_width/2]';
    
    % Define wheels
    front_wheel = [-wheel_width/2 -wheel_width/4;
                   wheel_width/2  -wheel_width/4;
                   wheel_width/2   wheel_width/4;
                   -wheel_width/2  wheel_width/4]';
    
    rear_wheel_left = front_wheel;
    rear_wheel_right = front_wheel;
    
    % Transform body
    body_trans = R * body;
    body_trans(1,:) = body_trans(1,:) + x;
    body_trans(2,:) = body_trans(2,:) + y;
    
    % Transform front wheel
    front_wheel = R * front_wheel;
    front_wheel(1,:) = front_wheel(1,:) + x + params.L/2*cos(theta);
    front_wheel(2,:) = front_wheel(2,:) + y + params.L/2*sin(theta);
    
    % Transform rear wheels
    rear_wheel_left = R * rear_wheel_left;
    rear_wheel_left(1,:) = rear_wheel_left(1,:) + x - params.L/2*cos(theta) - body_width/2*sin(theta);
    rear_wheel_left(2,:) = rear_wheel_left(2,:) + y - params.L/2*sin(theta) + body_width/2*cos(theta);
    
    rear_wheel_right = R * rear_wheel_right;
    rear_wheel_right(1,:) = rear_wheel_right(1,:) + x - params.L/2*cos(theta) + body_width/2*sin(theta);
    rear_wheel_right(2,:) = rear_wheel_right(2,:) + y - params.L/2*sin(theta) - body_width/2*cos(theta);
    
    % Plot vehicle
    fill(body_trans(1,:), body_trans(2,:), 'b', 'FaceAlpha', 0.3)  % Vehicle body
    fill(front_wheel(1,:), front_wheel(2,:), 'k')  % Front wheel
    fill(rear_wheel_left(1,:), rear_wheel_left(2,:), 'k')  % Rear left wheel
    fill(rear_wheel_right(1,:), rear_wheel_right(2,:), 'k')  % Rear right wheel
end

function plot_results(t, states, track, deltaout, erroryout, errorpsiout)
    figure('Name', 'Tricycle Tracking Results')
    
    % Position trajectory
    subplot(3,1,1)
    plot(states(1,:), states(2,:), 'b-')
    hold on
    plot(track.x, track.y, 'r--')
    grid on
    xlabel('X Position [m]')
    ylabel('Y Position [m]')
    title('Vehicle Trajectory')
    legend('Actual Path', 'Desired Path')
    
    % Steering angle
    subplot(3,1,2)
    plot(t, deltaout)
    grid on
    xlabel('Time [s]')
    ylabel('Steering Angle [rad]')
    title('Steering Angle vs Time')
    
    % Tracking errors
    subplot(3,1,3)
    plot(t, erroryout, 'b-', t, errorpsiout, 'r-')
    grid on
    xlabel('Time [s]')
    ylabel('Error')
    title('Tracking Errors')
    legend('Lateral Error [m]', 'Heading Error [rad]')
end