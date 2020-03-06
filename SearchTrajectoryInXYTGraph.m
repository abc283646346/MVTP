function [x, y, theta, quality] = SearchTrajectoryInXYTGraph(bv)
global xyt_graph_search_ environment_scale_
start_ind = ConvertConfigToIndex(bv.x0, bv.y0, 0);
goal_ind = ConvertConfigToIndex(bv.xtf, bv.ytf, xyt_graph_search_.t_max);
[ind_vec, theta, quality] = SearchViaAStar(start_ind, goal_ind, bv.theta0);
len = size(ind_vec,1);
while (len < xyt_graph_search_.num_nodes_t)
    disp('Not good ig!');
    ind_vec = [ind_vec; ind_vec(len,:)];
    theta = [theta, theta(len)];
    len = size(ind_vec,1);
end
x = environment_scale_.x_min + (ind_vec(:,1)' - 1) .* xyt_graph_search_.resolution_x;
y = environment_scale_.y_min + (ind_vec(:,2)' - 1) .* xyt_graph_search_.resolution_y;
if (length(x) ~= xyt_graph_search_.num_nodes_t)
    disp('[SearchTrajectoryInXYTGraph] ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
end
if (length(theta) ~= xyt_graph_search_.num_nodes_t)
    disp('[SearchTrajectoryInXYTGraph] ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
end
end

function [ind_vec, theta, quality] = SearchViaAStar(start_ind, goal_ind, theta0)
global xyt_graph_search_
grid_space_3D = cell(xyt_graph_search_.num_nodes_x, xyt_graph_search_.num_nodes_y, xyt_graph_search_.num_nodes_t);
% Information of each element in each node:
%  Dim # |  Variable
%  1-3      index of current node
%  4        f
%  5        g
%  6        h
%  7        is_in_openlist
%  8        is_in_closedlist
%  9-11     index of parent node
%  12-14    parent node's expansion vector
%  15       orientation angle
init_node(1:3) = start_ind;
init_node(5) = 0;
init_node(6) = sum(abs(start_ind(1:2) - goal_ind(1:2))) + xyt_graph_search_.weight_for_time * abs(start_ind(3) - goal_ind(3));
init_node(4) = xyt_graph_search_.multiplier_for_H * init_node(6);
init_node(7) = 1;
init_node(8) = 0;
init_node(9:11) = [start_ind(1) start_ind(2) -999];
init_node(12:14) = [0 0 0];
init_node(15) = theta0;
openlist_ = init_node;
grid_space_3D{init_node(1), init_node(2), init_node(3)} = init_node;
cur_best_node = init_node;
expansion_pattern = [
    1  1  1;
    1  0  1;
    1 -1  1;
    0  1  1;
    0  0  1;
    0 -1  1;
    -1  1  1;
    -1  0  1;
    -1 -1  1
    ];
length_type_1 = sqrt(1^2 + 1^2 + xyt_graph_search_.weight_for_time^2);
length_type_2 = sqrt(1^2 + xyt_graph_search_.weight_for_time^2);
length_type_3 = abs(xyt_graph_search_.weight_for_time);
expansion_length = [
    length_type_1;
    length_type_2;
    length_type_1;
    length_type_2;
    length_type_3;
    length_type_2;
    length_type_1;
    length_type_2;
    length_type_1
    ];
completeness_flag = 0;

iter = 0;
while ((~isempty(openlist_))&&(iter <= xyt_graph_search_.max_cycle)&&(~completeness_flag))
    iter = iter + 1;
    cur_node_order = find(openlist_(:,4) == min(openlist_(:,4))); cur_node_order = cur_node_order(end);
    cur_node = openlist_(cur_node_order, :);
    cur_ind = cur_node(1:3);
    cur_g = cur_node(5);
    cur_operation = cur_node(12:14);
    cur_theta = cur_node(15);
    % Remove cur_node from open list and close it
    openlist_(cur_node_order, :) = [];
    grid_space_3D{cur_ind(1), cur_ind(2), cur_ind(3)}(7) = 0;
    grid_space_3D{cur_ind(1), cur_ind(2), cur_ind(3)}(8) = 1;
    for ii = 1 : size(expansion_pattern,1)
        child_node_ind = cur_ind + expansion_pattern(ii,:);
        child_node_theta = CalculateTheta(cur_theta, expansion_pattern(ii,1:2));
        if ((child_node_ind(1) < 1)||(child_node_ind(2) < 1)||...
                (child_node_ind(3) < 1)||(child_node_ind(1) > xyt_graph_search_.num_nodes_x)||...
                (child_node_ind(2) > xyt_graph_search_.num_nodes_y)||...
                (child_node_ind(3) > xyt_graph_search_.num_nodes_t))
            continue;
        end
        % If the child node has been explored ever before, and then if the
        % child has been within the closed list, abandon it and continue.
        if ((~isempty(grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)}))...
                &&(grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)}(8) == 1))
            continue;
        end
        child_g = cur_g + expansion_length(ii,1) + 0 * sum(abs(expansion_pattern(ii,1:2) - cur_operation(1:2))); % 0.25
        child_h = sum(abs(child_node_ind(1:2) - goal_ind(1:2))) + xyt_graph_search_.weight_for_time * abs(child_node_ind(3) - goal_ind(3));
        child_f = child_g + xyt_graph_search_.multiplier_for_H * child_h;
        child_node_prepare = [child_node_ind, child_f, child_g, child_h, 1, 0, cur_ind, expansion_pattern(ii,:), child_node_theta];
        % If the child node has been explored ever before (but not closed yet)
        if (~isempty(grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)}))
            % The child must be in the open list now, then check if its
            % recorded parent deserves to be switched as our cur_node.
            if (grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)}(5) > child_g + 0.01)
                child_node_order1 = find(openlist_(:,1) == child_node_ind(1));
                child_node_order2 = find(openlist_(child_node_order1,2) == child_node_ind(2));
                child_node_order3 = find(openlist_(child_node_order1(child_node_order2),3) == child_node_ind(3));
                openlist_(child_node_order1(child_node_order2(child_node_order3)), :) = [];
                grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node_prepare;
                openlist_ = [openlist_; child_node_prepare];
            end
        else % Child node has never been explored before
            % If the child node is collison free
            if (IsNodeValid(child_node_ind, child_node_theta))
                openlist_ = [openlist_; child_node_prepare];
                grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node_prepare;
                if (~any(child_node_ind - goal_ind))
                    cur_best_node = child_node_prepare;
                    quality = child_g;
                    completeness_flag = 1;
                    break;
                end
                if (child_h < cur_best_node(6))
                    cur_best_node = child_node_prepare;
                end
            else % If the child node involves collisons
                child_node_prepare(8) = 1;
                child_node_prepare(7) = 0;
                grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node_prepare;
            end
        end
    end
end
ind_vec = cur_best_node(1:3);
theta = cur_best_node(15);
parent_ind = grid_space_3D{cur_best_node(1), cur_best_node(2), cur_best_node(3)}(9:11);
while (parent_ind(3) ~= -999)
    ind_vec = [parent_ind; ind_vec];
    theta = [grid_space_3D{parent_ind(1), parent_ind(2), parent_ind(3)}(15), theta];
    parent_ind = grid_space_3D{parent_ind(1), parent_ind(2), parent_ind(3)}(9:11);
end
if (~completeness_flag)
    quality = xyt_graph_search_.n_large_value + cur_best_node(6);
end
end

function child_node_theta = CalculateTheta(cur_theta, expansion_pattern)
if ((expansion_pattern(1) == 0)&&(expansion_pattern(2) == 0))
    child_node_theta = cur_theta;
    return;
end
global xyt_graph_search_
child_node_theta = atan2(xyt_graph_search_.resolution_y * expansion_pattern(2), xyt_graph_search_.resolution_x * expansion_pattern(1));
while (child_node_theta - cur_theta > 0.5 * pi + 0.0001)
    child_node_theta = child_node_theta - pi;
end
while (child_node_theta - cur_theta < -0.5 * pi - 0.0001)
    child_node_theta = child_node_theta + pi;
end
end

function is_valid = IsNodeValid(ind, theta)
global xyt_graph_search_ environment_scale_ vehicle_geometrics_
x = environment_scale_.x_min + xyt_graph_search_.resolution_x * (ind(1) - 1);
y = environment_scale_.y_min + xyt_graph_search_.resolution_y * (ind(2) - 1);
xr = x +  vehicle_geometrics_.r2p * cos(theta);
yr = y +  vehicle_geometrics_.r2p * sin(theta);
xf = x +  vehicle_geometrics_.f2p * cos(theta);
yf = y +  vehicle_geometrics_.f2p * sin(theta);
R = vehicle_geometrics_.radius;

global original_obstacle_layers_
xx = original_obstacle_layers_{1,ind(3)}.x;
yy = original_obstacle_layers_{1,ind(3)}.y;

if (IsAnyPointCollidingWithDualCircles(xr, yr, xf, yf, R, xx, yy))
    is_valid = 0;
else
    is_valid = 1;
end
end

function is_colliding = IsAnyPointCollidingWithDualCircles(xr, yr, xf, yf, R, xx, yy)
xx_backup = xx; yy_backup = yy;
ind = find(abs(xx - xr) + abs(yy - yr) > 1.4142 * R);
xx(ind) = []; yy(ind) = [];
err_x = xx - xr; err_y = yy - yr;
ind = find(err_x.^2 + err_y.^2 < R^2);
if (length(ind))
    is_colliding = 1;
    return;
end

ind = find(abs(xx_backup - xf) + abs(yy_backup - yf) > 1.4142 * R);
xx_backup(ind) = []; yy_backup(ind) = [];
err_x = xx_backup - xf; err_y = yy_backup - yf;
ind = find(err_x.^2 + err_y.^2 < R^2);
if (length(ind))
    is_colliding = 1;
    return;
end
is_colliding = 0;
end

function ind = ConvertConfigToIndex(x, y, time)
global xyt_graph_search_ environment_scale_
ind1 = round((x - environment_scale_.x_min) / xyt_graph_search_.resolution_x) + 1;
ind2 = round((y - environment_scale_.y_min) / xyt_graph_search_.resolution_y) + 1;
ind3 = round(time / xyt_graph_search_.resolution_t) + 1;
if (ind1 < 1)
    ind1 = 1;
elseif (ind1 > xyt_graph_search_.num_nodes_x)
    ind1 = xyt_graph_search_.num_nodes_x;
end
if (ind2 < 1)
    ind2 = 1;
elseif (ind2 > xyt_graph_search_.num_nodes_y)
    ind2 = xyt_graph_search_.num_nodes_y;
end
if (ind3 < 1)
    ind3 = 1;
elseif (ind3 > xyt_graph_search_.num_nodes_t)
    ind3 = xyt_graph_search_.num_nodes_t;
end
ind = [ind1, ind2, ind3];
end