classdef RobotEKF < handle
    properties
        % State vector: [x, y, theta]
        state
        % Covariance matrix
        P
        
        % Sensor noise covariances
        R_camera  % Camera measurement noise
        R_encoder % Encoder measurement noise
        
        % Process noise covariance
        Q
        
        % Sampling time
        dt
    end
    
    methods
        function obj = RobotEKF(initial_state, dt)
            % Constructor
            obj.state = initial_state;  % [x, y, theta]
            obj.dt = dt;
            
            % Initialize covariance matrix (high initial uncertainty)
            obj.P = diag([10, 10, 0.1]);
            
            % Define sensor noise covariances
            % Lower values mean higher trust in that sensor
            obj.R_camera = diag([0.5, 0.5, 0.01]);   % Camera: more precise
            obj.R_encoder = diag([2, 2, 0.1]);       % Encoder: less precise
            
            % Process noise (model uncertainty)
            obj.Q = diag([0.1, 0.1, 0.01]);
        end
        
        function [state, covariance] = predict(obj, encoder_distance, imu_angle)
            % Non-linear motion model prediction step
            
            % Current state
            x = obj.state(1);
            y = obj.state(2);
            theta = obj.state(3);
            
            % Predict new state using encoder and IMU
            dx = encoder_distance * cos(theta);
            dy = encoder_distance * sin(theta);
            
            obj.state(1) = x + dx;
            obj.state(2) = y + dy;
            obj.state(3) = imu_angle;
            
            % Compute Jacobian of motion model
            F = [1, 0, -dx*sin(theta);
                 0, 1,  dy*cos(theta);
                 0, 0,  1];
            
            % Predict covariance
            obj.P = F * obj.P * F' + obj.Q;
            
            state = obj.state;
            covariance = obj.P;
        end
        
        function [state, covariance] = updateWithCamera(obj, camera_coords)
            % Camera measurement update step
            
            % Measurement innovation
            z_innov = [camera_coords(1) - obj.state(1);
                       camera_coords(2) - obj.state(2);
                       camera_coords(3) - obj.state(3)];
            
            % Measurement Jacobian
            H = eye(3);
            
            % Kalman Gain
            S = H * obj.P * H' + obj.R_camera;
            K = obj.P * H' / S;
            
            % State update
            obj.state = obj.state + K * z_innov;
            
            % Covariance update
            obj.P = (eye(3) - K*H) * obj.P;
            
            state = obj.state;
            covariance = obj.P;
        end
    end
end

% Example usage script
function main()
    % Initialize EKF
    ekf = RobotEKF([0, 0, 0], 0.05);  % Initial state, sampling time
    
    % Simulated data loop
    for i = 1:1000
        % Simulated encoder and IMU data
        encoder_distance = 0.1;  % 10 cm movement
        imu_angle = rand() * pi/18;  % Small angle variation
        
        % Prediction step
        [predicted_state, predicted_cov] = ekf.predict(encoder_distance, imu_angle);
        
        % Every 20 iterations (simulating camera update)
        if mod(i, 20) == 0
            camera_coords = [predicted_state(1) + randn()*0.5, ...
                             predicted_state(2) + randn()*0.5, ...
                             predicted_state(3) + randn()*0.1];
            
            [fused_state, fused_cov] = ekf.updateWithCamera(camera_coords);
            
            % Visualization or logging could be added here
        end
    end
end