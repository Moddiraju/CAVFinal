% --- MAIN OPTIMIZATION SCRIPT ---
clear; clc;

% 1. Setup Initial Guess [kp_lat, kd_lat, kp_head, kd_head]
% Removed the 5th extra value from your original guess
initial_guess = [0.2, 0.1, 0.1, 0.2, 0.02, 0.5]; 

% 2. Setup Pattern Search Options 
% This replaces optimset. 'UseParallel' is great if you have a multi-core CPU.
options = optimoptions('patternsearch', ...
    'Display', 'iter', ...
    'FunctionTolerance', 1e-1, ...
    'StepTolerance', 1e-3, ...
    'MaxIterations', 100, ...
    'PlotFcn', @psplotbestf); % This gives you a nice live graph of the cost

disp('Starting Pattern Search Optimization...');

% 3. Run the Solver
% patternsearch handles the "cliffs" of Simulink much better than fminsearch

% 1. Define Lower Bounds (Gains should never be negative)
lb = [0.0, 0.03, 0.0, 0.0, 0.075, 0.0]; 

% 2. Define Upper Bounds (Adjust based on what feels sane for your car)
% [kp_lat, kd_lat, ki_lat, kp_head, kd_head]
ub = [0.2, 2.0, 1.0, 3.0, 1.0, 10.0]; 

% 3. Run the Solver with bounds
[optimal_gains, final_cost] = patternsearch(@vehicle_cost_function, initial_guess, ...
    [], [], [], [], lb, ub, [], options);

% 4. Display Results
fprintf('\n=================================\n');
fprintf('Optimization Complete!\n');
fprintf('Optimal kp_lat (Lateral):   %.4f\n', optimal_gains(1));
fprintf('Optimal kd_lat (Lat Rate):  %.4f\n', optimal_gains(2));
fprintf('Optimal ki_lat (Lat Error):  %.4f\n', optimal_gains(3));
fprintf('Optimal kp_head (Heading):  %.4f\n', optimal_gains(4));
fprintf('Optimal kd_head (Head Rate):%.4f\n', optimal_gains(5));
fprintf('Optimal kp_ff (Feedfwd):%.4f\n', optimal_gains(6));
fprintf('Final Cost:                 %.4f\n', final_cost);
fprintf('=================================\n');

% --- THE OBJECTIVE (COST) FUNCTION ---
function cost = vehicle_cost_function(gains)
    % Assign gains to base workspace for Simulink blocks
    assignin('base', 'kp_lat', gains(1));
    assignin('base', 'kd_lat', gains(2));
    assignin('base', 'ki_lat',  gains(3));
    assignin('base', 'kp_head', gains(4));
    assignin('base', 'kd_head', gains(5));
    assignin('base', 'kp_ff', gains(6));

    try
        out = sim('CAVFinal', 'CaptureErrors', 'on', 'StopTime', '20');
        if ~isempty(out.ErrorMessage)
            cost = 1e8; return;
        end
        
        % 1. Extract ALL relevant signals
        lat_vals   = out.get('lat_error');  % Distance from path
        head_vals  = out.get('heading_error'); % Angle from path
        steer_vals = out.get('steering_cmd');  % Raw steering wheel angle
        
        t = linspace(0, 20, length(lat_vals))'; 
        
        % 2. Calculate individual ITAE costs
        % We multiply heading by a weight (e.g., 10) because radians are 
        % numerically much smaller than meters, but just as important!
        lat_cost  = sum(t .* abs(lat_vals));
        head_cost = sum(t .* abs(head_vals) * 10); 
        
        % 3. Control Effort Penalty (Keep the steering wheel smooth)
        % We just sum the square of the steering to punish jerky movements
        steer_cost = sum(steer_vals.^2) * 50;
        
        % Total Cost: Balance of Position, Angle, and Smoothness
        cost = lat_cost + head_cost + steer_cost;

    catch
        cost = 1e8;
    end
end