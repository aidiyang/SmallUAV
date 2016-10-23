% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RB 
%   5/14/2010 - RB
%   2016/9/27 - EDISON YANG
%   Remake : in real MAV system this file sensors.m is unnecessary
%            this is the simulation compute modeling for sensors and compute
%            data to use simulation

function y = sensors(uu, P)

    % relabel the inputs
%     pn      = uu(1);
%     pe      = uu(2);
    pd      = uu(3);
    u       = uu(4);
    v       = uu(5);
    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
%     psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
%     M_l     = uu(16);
%     M_m     = uu(17);
%     M_n     = uu(18);
    Va      = uu(19);
%     alpha   = uu(20);
%     beta    = uu(21);
%     wn      = uu(22);
%     we      = uu(23);
%     wd      = uu(24);
    
    % simulate rate gyros (units are rad/sec)    %%%Eat represent zero-mean Gaussian processes 
    y_gyro_x = p + eat_gyro_x;
    y_gyro_y = q + eat_gyro_y;
    y_gyro_z = r + eat_gyro_z;

    % simulate accelerometers (units of g)
    eat_acce_x = 10^(-1)*rand(1);  %%% get zero-mean Gaussian noise
    eat_acce_x = 10^(-1)*rand(1);
    eat_acce_x = 10^(-1)*rand(1);
    y_accel_x = F_x/P.mass + q*w - r*v + P.gravity*sin(theta) + eat_acce_x;
    y_accel_y = F_y/P.mass + r*u - p*w - P.gravity*cos(theta)*sin(phi) + eat_acce_y;
    y_accel_z = F_z/P.mass + p*v - q*u - P.gravity*cos(theta)*cos(phi) + eat_acce_z;

    % simulate pressure sensors
    y_static_pres = P.rho * P.gravity *pd + bata_abs_pres + eat_abs_pres; %%% h_AGL = pd
    y_diff_pres = P.rho * Va^2/2 + beta_diff_pres +eat_diff_pres;%%% eat_diff_pres is zero-mean Gaussian noise %%%beta_diff_pres is temperature-related bias drift
    

    % construct total output
    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



