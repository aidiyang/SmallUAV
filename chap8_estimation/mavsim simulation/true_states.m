function xhat = true_states(uu,P)
%
% fake state estimation for mavsim
%   - this function will be replaced with a state estimator in a later
%   chapter.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   thetahat - estimated pitch angel, 
%   qhat     - estimated pitch rate, 
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   byhat    - estimate of y-gyro bias
% 
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   11/18/2014 - RWB
%   

    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    h        = -uu(2+NN); % altitude
    theta    = uu(5+NN);  % pitch angle
    q        = uu(6+NN); % body frame pitch rate
    NN = NN+6;
    Va       = uu(1+NN);  % airspeed
    alpha    = uu(2+NN);  % angle of attack
    wn       = uu(3+NN);  % wind North
    wd       = uu(4+NN);  % wind Down
    NN = NN+4;
    t        = uu(1+NN);   % time
    
    % estimate states (using real state data)
    pnhat    = pn;
    hhat     = h;
    Vahat    = Va;
    alphahat = alpha;
    thetahat = theta;
    qhat     = q;
    gamma_a = theta-alpha;
    Vghat    = sqrt((Va*cos(gamma_a)+wn)^2 + (-Va*sin(gamma_a)+wd)^2);
    wnhat    = wn;
    byhat    = P.bias_gyro_y;
    
    xhat = [...
        pnhat;...
        hhat;...
        Vahat;...
        alphahat;...
        thetahat;...
        qhat;...
        Vghat;...
        wnhat;...
        byhat;...
        ];
    
end 