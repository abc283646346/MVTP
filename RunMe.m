% ======================================================================= %
%  MATLAB Source Codes for article "Cooperative Trajectory Planning For   %
%  Multiple Nonholonomic Vehicles in Dynamic Scenarios: A Tractable       %
%  Optimization Method".                                                  %
%  Copyright (C) 2020 Bai Li                                              %
%  2020.05.28                                                             %
% ======================================================================= %
clear; close all; clc;

% % Basic Parametric Settings
global params_
params_.wheelbase = 2.8;
params_.front_hang = 0.96;
params_.rear_hang = 0.929;
params_.width = 1.942;
params_.length = params_.wheelbase + params_.front_hang + params_.rear_hang;
params_.radius = hypot(0.25 * params_.length, 0.5 * params_.width);
params_.r2p = 0.25 * params_.length - params_.rear_hang;
params_.f2p = 0.75 * params_.length - params_.rear_hang;

params_.v_max = 2.5;
params_.a_max = 0.5;
params_.phy_max = 0.7;
params_.w_max = 0.5;

params_.x_min = -20;
params_.x_max = 20;
params_.y_min = -20;
params_.y_max = 20;
params_.x_scale = params_.x_max - params_.x_min;
params_.y_scale = params_.y_max - params_.y_min;

params_.tf = 40;
params_.num_nodes_x = 150;
params_.num_nodes_y = 150;
params_.num_nodes_t = 200;
params_.resolution_t = params_.tf / (params_.num_nodes_t - 1);
params_.resolution_x = params_.x_scale / (params_.num_nodes_x - 1);
params_.resolution_y = params_.y_scale / (params_.num_nodes_y - 1);
params_.multiplier_for_H = 2.0;
params_.weight_for_time = 5.0;
params_.max_cycle = 1000;
params_.n_large_value = 10000000;
params_.Rcos_ang = params_.radius .* cos(linspace(0, 2 * pi, 30));
params_.Rsin_ang = params_.radius .* sin(linspace(0, 2 * pi, 30));

params_.Nfe = 100;

params_.norm_L_threshold = 5;
params_.L_threshold = params_.norm_L_threshold;
params_.alpha = 0.5;
params_.Iter_max = 10;

% % Tests on 400 benchmark cases
num_cases = 400;
success_flag = zeros(1, num_cases);
cpu_time = zeros(1, num_cases);

for case_id = 9 : 9
    clc; case_str = ['BenchmarkCases\', num2str(case_id)];
    global boundary_configs_ obstacles_ original_obstacle_layers_
    load(case_str); tic;
    WriteBoundaryValuesAndBasicParams(Nv, Nobs);
    % % SASUM Begins
    % Stage 2
    [init_x, init_y, init_theta, init_v, init_a, init_phy, init_w] = GenerateInitialGuess(Nv);
    % Stage 3
    iter = 0; fail_flag = 0;
    while (~IsCollisionFree(init_x, init_y, init_theta))
        if (iter > params_.Iter_max)
            fail_flag = 1;
            break;
        end
        WriteObstaclesForReducedNLP(init_x, init_y, init_theta, Nv);
        !ampl rr.run
        load opti_flag.txt
        if (~opti_flag)
            params_.L_threshold = params_.L_threshold * params_.alpha;
        else
            [init_x, init_y, init_theta, init_phy] = UpdateInitialGuess(Nv);
            params_.L_threshold = params_.norm_L_threshold;
        end
        iter = iter + 1;
    end
    
    cpu_time(case_id) = toc;
    if (~fail_flag)
        success_flag(case_id) = 1;
        GenerateVideo(init_x, init_y, init_theta, init_phy);
    else
        success_flag(case_id) = 0;
    end
    % % SASUM Ends
end