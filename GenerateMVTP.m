function [bv, obs] = GenerateMVTP(Nv, Nobs)
global environment_scale_
elem.x0 = environment_scale_.x_min + environment_scale_.x_scale * rand;
elem.y0 = environment_scale_.y_min + environment_scale_.y_scale * rand;
elem.xtf = environment_scale_.x_min + environment_scale_.x_scale * rand;
elem.ytf = environment_scale_.y_min + environment_scale_.y_scale * rand;
elem.theta0 = -pi + 2 * pi * rand;
elem.thetatf = -pi + 2 * pi * rand;
while ((hypot(elem.x0 - elem.xtf, elem.y0 - elem.ytf) > 50)||(hypot(elem.x0 - elem.xtf, elem.y0 - elem.ytf) < 10))
    elem.xtf = environment_scale_.x_min + environment_scale_.x_scale * rand;
    elem.ytf = environment_scale_.y_min + environment_scale_.y_scale * rand;
    elem.thetatf = -pi + 2 * pi * rand;
end
bv = cell(1,1); bv{1,1} = elem;
for ii = 2 : Nv
    while (1)
        elem.x0 = environment_scale_.x_min + environment_scale_.x_scale * rand;
        elem.y0 = environment_scale_.y_min + environment_scale_.y_scale * rand;
        elem.theta0 = -pi + 2 * pi * rand;
        elem.xtf = environment_scale_.x_min + environment_scale_.x_scale * rand;
        elem.ytf = environment_scale_.y_min + environment_scale_.y_scale * rand;
        elem.thetatf = -pi + 2 * pi * rand;
        while ((hypot(elem.x0 - elem.xtf, elem.y0 - elem.ytf) > 20)||(hypot(elem.x0 - elem.xtf, elem.y0 - elem.ytf) < 10))
            elem.xtf = environment_scale_.x_min + environment_scale_.x_scale * rand;
            elem.ytf = environment_scale_.y_min + environment_scale_.y_scale * rand;
            elem.thetatf = -pi + 2 * pi * rand;
        end
        is_invalid = 0;
        for jj = 1 : (ii - 1)
            if (IsConflictingBetweenTwoConfigs(elem, bv{1,jj}))
                is_invalid = 1;
                break;
            end
        end
        if (is_invalid == 0)
            bv{1,ii} = elem;
            break;
        end
    end
end

obs = cell(1, Nobs);
ii = 0;
while (1)
    elem = GenerateRandomObstacleInformation();
    if (IsObsCollidingWithConfigs(elem, bv))
        continue;
    end
    ii = ii + 1;
    obs{1, ii} = elem;
    if (ii == Nobs)
        break;
    end
end
end

function is_collided = IsConflictingBetweenTwoConfigs(elem1, elem2)
is_collided = 1;
if (IsConfig1CollidingWithConfig2([elem1.x0, elem1.y0, elem1.theta0], [elem2.x0, elem2.y0, elem2.theta0]))
    return;
end
if (IsConfig1CollidingWithConfig2([elem1.xtf, elem1.ytf, elem1.thetatf], [elem2.xtf, elem2.ytf, elem2.thetatf]))
    return;
end
is_collided = 0;
end

function is_collided = IsConfig1CollidingWithConfig2(vec1, vec2)
is_collided = 1;
global vehicle_geometrics_
x1 = vec1(1); y1 = vec1(2); theta1 = vec1(3);
xr1 = x1 + vehicle_geometrics_.r2p * cos(theta1);
yr1 = y1 + vehicle_geometrics_.r2p * sin(theta1);
xf1 = x1 + vehicle_geometrics_.f2p * cos(theta1);
yf1 = y1 + vehicle_geometrics_.f2p * sin(theta1);
x2 = vec2(1); y2 = vec2(2); theta2 = vec2(3);
xr2 = x2 + vehicle_geometrics_.r2p * cos(theta2);
yr2 = y2 + vehicle_geometrics_.r2p * sin(theta2);
xf2 = x2 + vehicle_geometrics_.f2p * cos(theta2);
yf2 = y2 + vehicle_geometrics_.f2p * sin(theta2);
R = vehicle_geometrics_.radius;

buff = 1.2;
% r1 vs r2
if ((xr1 - xr2)^2 + (yr1 - yr2)^2 < buff * 4 * R^2)
    return;
end
% r1 vs f2
if ((xr1 - xf2)^2 + (yr1 - yf2)^2 < buff * 4 * R^2)
    return;
end
% f1 vs r2
if ((xf1 - xr2)^2 + (yf1 - yr2)^2 < buff * 4 * R^2)
    return;
end
% f1 vs f2
if ((xf1 - xf2)^2 + (yf1 - yf2)^2 < buff * 4 * R^2)
    return;
end
is_collided = 0;
end

function frame = GenerateRandomObstacleInformation()
global environment_scale_ xyt_graph_search_
x0 = environment_scale_.x_min + environment_scale_.x_scale * rand;
y0 = environment_scale_.y_min + environment_scale_.y_scale * rand;
if (rand < 0.2)
    dx = 0;
    dy = 0;
else
    dx = environment_scale_.x_scale / xyt_graph_search_.num_nodes_t * randn * 0.25;
    dy = environment_scale_.y_scale / xyt_graph_search_.num_nodes_t * randn * 0.25;
end
R = 1 + rand;
frame = cell(1, xyt_graph_search_.num_nodes_t);
for ii = 1 : xyt_graph_search_.num_nodes_t
    single_frame.x = x0 + dx * ii; single_frame.y = y0 + dy * ii; single_frame.radius = R;
    frame{1, ii} = single_frame;
end
end

function is_collided = IsObsCollidingWithConfigs(elem, bv)
is_collided = 1;
buff_ind_len = 10;
Nv = length(bv);
Nframe = length(elem);
for vv = 1 : Nv
    x0 = bv{1, vv}.x0; y0 = bv{1, vv}.y0; theta0 = bv{1, vv}.theta0;
    for ii = 1 : buff_ind_len
        xc = elem{1, ii}.x; yc = elem{1, ii}.y; R = elem{1, ii}.radius;
        if (IsConfigCollidingWithCircle([x0, y0, theta0], [xc, yc, R]))
            return;
        end
    end
    xtf = bv{1, vv}.xtf; ytf = bv{1, vv}.ytf; thetatf = bv{1, vv}.thetatf;
    for ii = (Nframe - buff_ind_len + 1) : Nframe
        xc = elem{1, ii}.x; yc = elem{1, ii}.y; R = elem{1, ii}.radius;
        if (IsConfigCollidingWithCircle([xtf, ytf, thetatf], [xc, yc, R]))
            return;
        end
    end
end
is_collided = 0;
end

function is_collided = IsConfigCollidingWithCircle(vec1, vec2)
is_collided = 1;
global vehicle_geometrics_
x1 = vec1(1); y1 = vec1(2); theta1 = vec1(3);
xr1 = x1 + vehicle_geometrics_.r2p * cos(theta1);
yr1 = y1 + vehicle_geometrics_.r2p * sin(theta1);
xf1 = x1 + vehicle_geometrics_.f2p * cos(theta1);
yf1 = y1 + vehicle_geometrics_.f2p * sin(theta1);
xc = vec2(1);
yc = vec2(2);

buff = 1.21;
% r1 vs c
if ((xr1 - xc)^2 + (yr1 - yc)^2 < buff * (vec2(3) + vehicle_geometrics_.radius)^2)
    return;
end
% f1 vs c
if ((xf1 - xc)^2 + (yf1 - yc)^2 < buff * (vec2(3) + vehicle_geometrics_.radius)^2)
    return;
end
is_collided = 0;
end