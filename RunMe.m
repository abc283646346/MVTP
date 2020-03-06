%  MATLAB Source Codes for article "Cooperative trajectory planning for
%  multiple nonholonomic vehicles with static and dynamic obstacles".
%  Copyright (C) 2020 Bai Li
%  2020.02.28
% ==============================================================================
%  Users of the source codes are suggested to cite the following papers.
%  a) Cooperative trajectory planning for multiple nonholonomic vehicles
%  with static and dynamic obstacles
%  b) Simultaneous versus joint computing: a case study of multi-vehicle
%  parking motion planning
% ==============================================================================
clear; close all; clc;

% % Parameters
global vehicle_geometrics_
vehicle_geometrics_.wheelbase = 2.8;
vehicle_geometrics_.front_hang = 0.96;
vehicle_geometrics_.rear_hang = 0.929;
vehicle_geometrics_.width = 1.942;
vehicle_geometrics_.length = vehicle_geometrics_.wheelbase + vehicle_geometrics_.front_hang + vehicle_geometrics_.rear_hang;
vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.length, 0.5 * vehicle_geometrics_.width);
vehicle_geometrics_.r2p = 0.25 * vehicle_geometrics_.length - vehicle_geometrics_.rear_hang;
vehicle_geometrics_.f2p = 0.75 * vehicle_geometrics_.length - vehicle_geometrics_.rear_hang;
global vehicle_kinematics_
vehicle_kinematics_.v_max = 2.5;
vehicle_kinematics_.a_max = 0.5;
vehicle_kinematics_.phy_max = 0.7;
vehicle_kinematics_.w_max = 0.5;
global environment_scale_
environment_scale_.x_min = -20;
environment_scale_.x_max = 20;
environment_scale_.y_min = -20;
environment_scale_.y_max = 20;
environment_scale_.x_scale = environment_scale_.x_max - environment_scale_.x_min;
environment_scale_.y_scale = environment_scale_.y_max - environment_scale_.y_min;
global xyt_graph_search_
xyt_graph_search_.t_max = 40;
xyt_graph_search_.num_nodes_x = 150;
xyt_graph_search_.num_nodes_y = 150;
xyt_graph_search_.num_nodes_t = 200;
xyt_graph_search_.resolution_t = xyt_graph_search_.t_max / (xyt_graph_search_.num_nodes_t - 1);
xyt_graph_search_.resolution_x = environment_scale_.x_scale / (xyt_graph_search_.num_nodes_x - 1);
xyt_graph_search_.resolution_y = environment_scale_.y_scale / (xyt_graph_search_.num_nodes_y - 1);
xyt_graph_search_.multiplier_for_H = 2.0;
xyt_graph_search_.weight_for_time = 5.0;
xyt_graph_search_.max_cycle = 1000;
xyt_graph_search_.n_large_value = 10000000;
ang = linspace(0, 2 * pi, 30);
xyt_graph_search_.Rcos_ang = vehicle_geometrics_.radius .* cos(ang);
xyt_graph_search_.Rsin_ang = vehicle_geometrics_.radius .* sin(ang);

global boundary_configs_ obstacles_
Nv = 5; Nobs = 5;
[boundary_configs_, obstacles_] = GenerateMVTP(Nv, Nobs);

% % Generate a number of priority order candidates
Norder = 18;
Nrank = min(Norder + 2, factorial(Nv));
ranklist = zeros(Nrank, Nv);
single_vehicle_travel_difficulty = zeros(1, Nv);
global original_obstacle_layers_
original_obstacle_layers_ = GenerateOriginalObstacleLayers();
backup_original_obstacle_layers = original_obstacle_layers_;
for ii = 1 : Nv
    [x, y, ~, single_vehicle_travel_difficulty(ii)] = SearchTrajectoryInXYTGraph(boundary_configs_{1,ii});
end
[~, ranklist(1,:)] = sort(single_vehicle_travel_difficulty);
[~, ranklist(2,:)] = sort(single_vehicle_travel_difficulty,'descend');
for ii = 3 : Nrank
    while (1) % Make sure the Nrank priority orders are mutually different
        list_candidate = randperm(Nv);
        if (prod(sum(abs(ranklist(1:(ii-1),:) - list_candidate)')))
            break;
        end
    end
    ranklist(ii,:) = list_candidate;
end
candidate_cost = zeros(1, Nrank);
Nfe = 100;
initial_guess = cell(1, Nrank);
for order_attempt = 1 : Nrank
    original_obstacle_layers_ = backup_original_obstacle_layers;
    decision_x = zeros(Nv, Nfe);
    decision_y = zeros(Nv, Nfe);
    decision_theta = zeros(Nv, Nfe);
    for ii = 1 : Nv
        iv = ranklist(order_attempt, ii);
        [x, y, theta, cost] = SearchTrajectoryInXYTGraph(boundary_configs_{1,iv});
        decision_x(iv, 1 : Nfe) = ResampleProfile(x, Nfe);
        decision_y(iv, 1 : Nfe) = ResampleProfile(y, Nfe);
        decision_theta(iv, 1 : Nfe) = ResampleProfile(theta, Nfe);
        UpdateObstacleLayers(x, y, theta);
        candidate_cost(1, order_attempt) = candidate_cost(1, order_attempt) + cost;
    end
    elem.x = decision_x;
    elem.y = decision_y;
    elem.theta = decision_theta;
    initial_guess{1, order_attempt} = elem;
end
[~,index] = sort(candidate_cost);

global NLP_
NLP_.L_threshold = vehicle_geometrics_.radius * 15;
NLP_.Nfe = Nfe;
NLP_.tf = xyt_graph_search_.t_max;
WriteBoundaryValuesAndBasicParams(Nv, Nfe, Nobs);

ready_flag = 0;
iter = 0;
while(~ready_flag)
    iter = iter + 1;
    if (iter > Nrank)
        break;
    end
    elem = initial_guess{1, iter};
    SpecifyInitialGuess(elem.x, elem.y, elem.theta);
    
    WriteObstaclesForReducedNLP(decision_x, decision_y, decision_theta, Nfe, Nv);
    !ampl rr.run
    load opti_flag.txt
    if ((CheckCollisions())||(~opti_flag))
        continue;
    end
    ready_flag = 1;
end
if (ready_flag)
    disp('Solution succeeded.');
    ShowDynamicResults();
else
    disp('Solution failed.');
end