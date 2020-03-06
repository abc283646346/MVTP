function WriteBoundaryValuesAndBasicParams(Nv, Nfe, Nobs)
warning off
global boundary_configs_
delete('BV');
fid = fopen('BV', 'w');
for ii = 1 : size(boundary_configs_,2)
    fprintf(fid, '%g 1  %f\r\n', ii, boundary_configs_{1,ii}.x0);
    fprintf(fid, '%g 2  %f\r\n', ii, boundary_configs_{1,ii}.y0);
    fprintf(fid, '%g 3  %f\r\n', ii, boundary_configs_{1,ii}.theta0);
    fprintf(fid, '%g 4  %f\r\n', ii, boundary_configs_{1,ii}.xtf);
    fprintf(fid, '%g 5  %f\r\n', ii, boundary_configs_{1,ii}.ytf);
    fprintf(fid, '%g 6  %f\r\n', ii, boundary_configs_{1,ii}.thetatf);
end
fclose(fid);

global vehicle_geometrics_ vehicle_kinematics_ xyt_graph_search_
delete('BP');
fid = fopen('BP', 'w');
fprintf(fid, '1  %f\r\n', vehicle_geometrics_.wheelbase);
fprintf(fid, '2  %f\r\n', vehicle_geometrics_.radius);
fprintf(fid, '3  %f\r\n', vehicle_geometrics_.r2p);
fprintf(fid, '4  %f\r\n', vehicle_geometrics_.f2p);

fprintf(fid, '5  %f\r\n', vehicle_kinematics_.v_max);
fprintf(fid, '6  %f\r\n', vehicle_kinematics_.a_max);
fprintf(fid, '7  %f\r\n', vehicle_kinematics_.phy_max);
fprintf(fid, '8  %f\r\n', vehicle_kinematics_.w_max);

fprintf(fid, '9  %f\r\n', xyt_graph_search_.t_max);
fprintf(fid, '10  %g\r\n', Nv);
fprintf(fid, '11  %g\r\n', Nfe);
fprintf(fid, '12  %g\r\n', Nobs);
fclose(fid);
end