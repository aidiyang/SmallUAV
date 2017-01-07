% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RB 
%   5/14/2010 - RB
%   11/18/2014 - RWB

function y = sensors(uu, P)

    % relabel the inputs
    NN = 0;
%    pn      = uu(1+NN);
    pd      = uu(2+NN);
%    u       = uu(3+NN);
%    w       = uu(4+NN);
    theta   = uu(5+NN);
    q       = uu(6+NN);
    NN = NN + 6;
    F_x     = uu(1+NN);
    F_z     = uu(2+NN);
%    M_m     = uu(3+NN);
    NN = NN + 3;
    Va      = uu(1+NN);
%    alpha   = uu(2+NN);
%    wn      = uu(3+NN);
%    wd      = uu(4+NN);
    
    % simulate rate gyros (units are rad/sec)
    y_gyro_y = q + P.bias_gyro_y + P.sigma_gyro*randn;

    % simulate accelerometers (units of g)
    y_accel_x = F_x/P.mass + P.gravity*sin(theta) + P.sigma_accel*randn;
    y_accel_z = F_z/P.mass - P.gravity*cos(theta) + P.sigma_accel*randn;

    % simulate pressure sensors
    y_static_pres = P.rho*P.gravity*(-pd) + P.sigma_static_pres*randn;
    y_diff_pres = 0.5*P.rho*Va^2 + P.sigma_diff_pres*randn;

    % construct total output
    y = [...
        y_gyro_y;...
        y_accel_x;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



