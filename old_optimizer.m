% --- MAIN OPTIMIZATION SCRIPT (PD-PD, PATTERN SEARCH) ---
clear; clc;

% 1. Initial Guess [kp_lat, kd_lat, kp_head, kd_head]
initial_guess = [0.2, 0.1, 0.001, 1.5, 0.05];

% 2. Pattern Search Options
options = optimoptions('patternsearch', ...
    'Display',           'iter', ...
    'FunctionTolerance', 1e-2, ...
    'StepTolerance',     1e-2, ...
    'MaxIterations',     100, ...
    'PlotFcn',           @psplotbestf);

% 3. Bounds
lb = [0, 0, 0, 0, 0];
ub = [2, 1, 1, 2, 1];

disp('Starting Pattern Search Optimization (PD-PD)...');

% 4. Run Solver
[optimal_gains, final_cost] = patternsearch(@vehicle_cost_function, initial_guess, ...
    [], [], [], [], lb, ub, [], options);

% 5. Push final gains to workspace
assignin('base', 'kp_lat',  optimal_gains(1));
assignin('base', 'kd_lat',  optimal_gains(2));
assignin('base', 'kp_head', optimal_gains(3));
assignin('base', 'kd_head', optimal_gains(4));

% 6. Display Results
fprintf('\n=================================\n');
fprintf('Optimization Complete!\n');
fprintf('Optimal kp_lat  (Lateral):   %.4f\n', optimal_gains(1));
fprintf('Optimal kd_lat  (Lat Rate):  %.4f\n', optimal_gains(2));
fprintf('Optimal kp_head (Heading):   %.4f\n', optimal_gains(3));
fprintf('Optimal kd_head (Head Rate): %.4f\n', optimal_gains(4));
fprintf('Final Cost:                  %.4f\n', final_cost);
fprintf('=================================\n');

% --- THE OBJECTIVE (COST) FUNCTION ---
function cost = vehicle_cost_function(gains)
    assignin('base', 'kp_lat',  gains(1));
    assignin('base', 'kd_lat',  gains(2));
    assignin('base', 'kp_head', gains(3));
    assignin('base', 'kd_head', gains(4));

    try
        out = sim('CAVFinal', 'CaptureErrors', 'on', 'StopTime', '20');

        if ~isempty(out.ErrorMessage)
            cost = 1e8; return;
        end

        % Extract signals
        error_vals   = out.get('lat_error');
        error_vals   = error_vals(:);
        steering     = out.get('steering_cmd');

        num_points = length(error_vals);

        % Exponential time weight (grows from 1 to ~e^10 ≈ 22000)
        t_normalized      = linspace(0, 1, num_points)';
        exponential_weight = exp(10 * t_normalized);

        % Weighted squared error cost
        weighted_errors = (error_vals.^2) .* exponential_weight;
        ise_weighted    = sum(weighted_errors);

        % Steering jerk penalty (punish rapid changes in steering)
        steering_effort = sum(diff(steering(:)).^2);

        % Final snap penalty (hard penalty if still >10cm off at t=20s)
        if abs(error_vals(end)) > 0.1
            final_snap_penalty = 1e6;
        else
            final_snap_penalty = 0;
        end

        sign_changes = sum(abs(diff(sign(error_vals)))) / 2;
        crossing_penalty = sign_changes * 1e6;

        cost = ise_weighted + (steering_effort * 100) + final_snap_penalty + crossing_penalty;

    catch
        cost = 1e8;
    end
end