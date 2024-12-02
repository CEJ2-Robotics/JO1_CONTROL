function robocar_animation(block)
    setup(block);
end

function setup(block)
    % number of ports
    block.NumInputPorts  = 3;  % Add a fourth port for waypoints
    block.NumOutputPorts = 0;
    
    block.SetPreCompInpPortInfoToDynamic;
    
    % parameters
    block.NumDialogPrms = 0;
    
    % sample times
    block.SampleTimes = [0.15 0];
    
    block.SimStateCompliance = 'DefaultSimState';
    
    % methods
    block.RegBlockMethod('Start', @start);
    block.RegBlockMethod('Outputs', @outputs);
    block.RegBlockMethod('Terminate', @terminate);
    block.RegBlockMethod('SetInputPortDimensions', @dims);
end

function start(~)
    persistent fig_handle;
end

function outputs(block)
    persistent fig_handle;

    state = block.InputPort(1).Data;
    L = block.InputPort(2).Data;
    waypoints = block.InputPort(3).Data;

    target = waypoints( length(waypoints), : );
    
    % check if figure still exists
    if isempty(fig_handle) || ~ishandle(fig_handle)
        fig_handle = figure('Name', 'Tricycle Animation');
        axis([0.5000  400.5000    0.5000  400.5000]);
        grid on;
        hold on;
        title('Tricycle Motion');
        xlabel('X Position [m]');
        ylabel('Y Position [m]');
    end
    
    figure(fig_handle);
    clf;
    hold on;
    grid on;
    
    % Plot waypoints with lines and points
    plot(waypoints(:,1), waypoints(:,2), 'g-o', 'MarkerSize', 4, 'DisplayName', 'Waypoints', 'LineWidth', 1);
    
    % call visualization function
    visualize_car(state, target, L);
    
    % update plot
    axis([0.5000  400.5000    0.5000  400.5000]);
    grid on;
    drawnow;
end

function terminate(~)
    % Clean up figure if needed
    persistent fig_handle;
    if ~isempty(fig_handle) && ishandle(fig_handle)
        close(fig_handle);
    end
    clear fig_handle;
end

function dims(block, idx, di)
    block.InputPort(idx).Dimensions = di;
end

function visualize_car(state, target, L)

    x = state(1);
    y = state(2);
    theta = state(3);

    % Tricycle visualization parameters
    wheel_width = L/5;
    body_width = L/2;
    
    % Create transformation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Define vehicle body points
    body = [-L/2 -body_width/2;
            L/2  -body_width/2;
            L/2   body_width/2;
            -L/2  body_width/2]';
    
    % Define front wheel
    front_wheel = [-wheel_width/2 -wheel_width/4;
                   wheel_width/2  -wheel_width/4;
                   wheel_width/2   wheel_width/4;
                   -wheel_width/2  wheel_width/4]';
    
    % Define rear wheels
    rear_wheel_left = front_wheel;
    rear_wheel_right = front_wheel;
    
    % Transform body
    body_trans = R * body;
    body_trans(1,:) = body_trans(1,:) + x;
    body_trans(2,:) = body_trans(2,:) + y;
    
    % Transform front wheel (with steering)
    front_wheel = R * front_wheel;
    front_wheel(1,:) = front_wheel(1,:) + x + L/2*cos(theta);
    front_wheel(2,:) = front_wheel(2,:) + y + L/2*sin(theta);
    
    % Transform rear wheels
    rear_wheel_left = R * rear_wheel_left;
    rear_wheel_left(1,:) = rear_wheel_left(1,:) + x - L/2*cos(theta) - body_width/2*sin(theta);
    rear_wheel_left(2,:) = rear_wheel_left(2,:) + y - L/2*sin(theta) + body_width/2*cos(theta);
    
    rear_wheel_right = R * rear_wheel_right;
    rear_wheel_right(1,:) = rear_wheel_right(1,:) + x - L/2*cos(theta) + body_width/2*sin(theta);
    rear_wheel_right(2,:) = rear_wheel_right(2,:) + y - L/2*sin(theta) - body_width/2*cos(theta);
    
    % Plot vehicle
    fill(body_trans(1,:), body_trans(2,:), 'b', 'FaceAlpha', 0.3)  % Vehicle body
    fill(front_wheel(1,:), front_wheel(2,:), 'k')  % Front wheel
    fill(rear_wheel_left(1,:), rear_wheel_left(2,:), 'k')  % Rear left wheel
    fill(rear_wheel_right(1,:), rear_wheel_right(2,:), 'k')  % Rear right wheel

    plot(target(1), target(2), 'r*', 'MarkerSize', 10)
end
