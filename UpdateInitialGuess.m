function [init_x, init_y, init_theta, init_phy] = UpdateInitialGuess(Nv)
global params_

load x.txt; load y.txt; load theta.txt; load phy.txt;
init_x = reshape(x', params_.Nfe, Nv)';
init_y = reshape(y', params_.Nfe, Nv)';
init_theta = reshape(theta', params_.Nfe, Nv)';
init_phy = reshape(phy', params_.Nfe, Nv)';
end