function WriteBoundaryValuesAndBasicParams(Nv, Nobs)
global params_ boundary_configs_

warning off
delete('BV');
fid = fopen('BV', 'w');
for ii = 1 : size(boundary_configs_,2)
    theta0 = boundary_configs_{1,ii}.theta0;
    thetatf = boundary_configs_{1,ii}.thetatf;
    while (thetatf - theta0 > pi + 0.01)
        thetatf = thetatf - 2 * pi;
    end
    while (thetatf - theta0 < -pi - 0.01)
        thetatf = thetatf + 2 * pi;
    end
    fprintf(fid, '%g 1  %f\r\n', ii, boundary_configs_{1,ii}.x0);
    fprintf(fid, '%g 2  %f\r\n', ii, boundary_configs_{1,ii}.y0);
    fprintf(fid, '%g 3  %f\r\n', ii, boundary_configs_{1,ii}.theta0);
    fprintf(fid, '%g 4  %f\r\n', ii, boundary_configs_{1,ii}.xtf);
    fprintf(fid, '%g 5  %f\r\n', ii, boundary_configs_{1,ii}.ytf);
    fprintf(fid, '%g 6  %f\r\n', ii, thetatf);
end
fclose(fid);

delete('BP');
fid = fopen('BP', 'w');
fprintf(fid, '1  %f\r\n', params_.wheelbase);
fprintf(fid, '2  %f\r\n', params_.radius);
fprintf(fid, '3  %f\r\n', params_.r2p);
fprintf(fid, '4  %f\r\n', params_.f2p);

fprintf(fid, '5  %f\r\n', params_.v_max);
fprintf(fid, '6  %f\r\n', params_.a_max);
fprintf(fid, '7  %f\r\n', params_.phy_max);
fprintf(fid, '8  %f\r\n', params_.w_max);

fprintf(fid, '9  %f\r\n', params_.tf);
fprintf(fid, '10  %g\r\n', Nv);
fprintf(fid, '11  %g\r\n', params_.Nfe);
fprintf(fid, '12  %g\r\n', Nobs);
fclose(fid);
end