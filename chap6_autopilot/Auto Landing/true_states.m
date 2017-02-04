function xhat = true_states(uu,P)
%
% fake state estimation for mavsim
%   - this function will be replaced with a state estimator in a later
%   chapter.
%
% Outputs are:(xhat 表示状态的最优估计值，这个表示来自估计值是x尖尖，带个帽子-hat)
%   pnhat    - estimated North position,    %导航系北东地 位置3向
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed,          %空速1
%   alphahat - estimated angle of attack,   %攻角1
%   betahat  - estimated sideslip angle,    %侧滑角1
%   phihat   - estimated roll angle,        %横滚角、俯仰角、偏航角3
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate,         %横滚角、俯仰角、偏航角 速度3
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed,      %地速对地速率，不是矢量1
%   wnhat    - estimate of North wind,      %风在北向和东的分量2
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle,   %机头角度1
%   bxhat    - estimate of x-gyro bias,     %陀螺仪的漂移3
%   byhat    - estimate of y-gyro bias,
%   bzhat    - estimate of z-gyro bias,     
% 
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   2017/1/20 - Edison Yang    注释

    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    pe       = uu(2+NN);  % inertial East position
    h        = -uu(3+NN); % altitude
%    u        = uu(4+NN);  % inertial velocity along body x-axis
%    v        = uu(5+NN);  % inertial velocity along body y-axis
%    w        = uu(6+NN);  % inertial velocity along body z-axis
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    psi      = uu(9+NN);  % yaw angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    NN = NN+12;
    Va       = uu(1+NN);  % airspeed
    alpha    = uu(2+NN);  % angle of attack
    beta     = uu(3+NN);  % sideslip angle
    wn       = uu(4+NN);  % wind North
    we       = uu(5+NN);  % wind East
%    wd       = uu(6+NN);  % wind down
    NN = NN+6;
    defl1    = uu(1+NN);
    defl2    = uu(2+NN);
    defl3    = uu(3+NN);
%    t        = uu(1+NN);   % time
    
    % estimate states (using real state data)
    pnhat    = pn;
    pehat    = pe;
    hhat     = h;
    Vahat    = Va;
    alphahat = alpha;
    betahat  = beta;
    phihat   = phi;
    thetahat = theta;
    chihat   = atan2(Va*sin(psi)+we, Va*cos(psi)+wn);
    phat     = p;
    qhat     = q;
    rhat     = r;
    Vghat    = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2);
    wnhat    = wn;
    wehat    = we;
    psihat   = psi;
%     bxhat    = P.bias_gyro_x;
%     byhat    = P.bias_gyro_y;
%     bzhat    = P.bias_gyro_z;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        defl1;...
        defl2;...
        defl3;...
        ];
    
end 