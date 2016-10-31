% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%-------------------------------------------------------
% by Edisonyang  Date:2016.09.03
%
% 算法来源：飞行器动力学模型
% NOTE: 完成了飞行器外部受力和力矩的方程。
%       其中:1.不同的飞机（固定翼和飞翼的，空气动力学参数不同，估计和没有垂尾有关系）
%           2.关于涵道发动机部分的建模后面要加进来。对涵道的力学建模问题像螺旋桨一样
%             利用贝努力方程分析来流和去流进行建模
% 修改说明:
% 2016.10.22 : 改进了变形函道发动机对机体力矩的表达式.
% 2016.10.25
%
function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    %Ducted fan engine---
    F1      = delta(5);
    F2      = delta(6);
    F3      = delta(7);
    F4      = delta(8);
    %------------------------
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % compute wind data in NED   //20160831 EDISONYANG
    % NED系下的风速＝NED系下稳定风速＋Cnb＊机体系下的风噪声
    w_n = w_ns+u_wg;
    w_e = w_es+v_wg;
    w_d = w_ds+w_wg;
    
%-----------------------------------------------------    
    %%% 导航系-》机体系
    T11=cos(theta)*cos(psi);        %T11=C11
    T12=cos(theta)*sin(psi);        %T12=C21
    T13=-sin(theta);                %T13=C31
    T21=sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);   %T21=C12
    T22=sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);   %T22=C22
    T23=sin(phi)*cos(theta);        %T23=C32
    T31=cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);   %T31=C13
    T32=cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);   %T32=C23
    T33=cos(phi)*cos(theta);        %T33=C33
%   Cbn=[T11,T12,T13;T21,T22,T23;T31,T32,T33]          %Cbn=Cnb'
%-----------------------------------------------------    
    %计算：机体系下的风速=Cbn*导航系下恒风+机体系下随机风（阵风）
    u_w = T11*w_ns + T12*w_es + T13*w_ds + u_wg;
    v_w = T21*w_ns + T22*w_es + T23*w_ds + v_wg;
    w_w = T31*w_ns + T32*w_es + T33*w_ds + w_wg;
    
    %计算：机体系下的空速=机系下飞机速度 - 机体系下风速
    u_r = u - u_w;
    v_r = v - v_w;
    w_r = w - w_w;
  
    % compute air data
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    alpha = atan(w_r/u_r);
    beta = atan(v_r/sqrt(u_r^2 + w_r^2));
    
% ------------------------------------------------------   
% compute external forces and torques on aircraft
  %1. gravity force in body frame 
    f_gx = - P.mss * P.gravity * sin(theta);
    f_gy = P.mss * P.gravity * cos(theta) * sin(phi);
    f_gz = P.mss * P.gravity * cos(theta) * cos(phi);
    
  %2. Aerodynamic Forces in body frame
    % parameter of Aerodynamic Forces  %定义中间变量
    C_X_alpha = - P.C_D_alpha * cos(alpha) + P.C_L_alpha * sin(alpha);  %define
    C_X_q_alpha = -P.C_D_q * cos(alpha) + P.C_L_q *sin(alpha);
    C_X_delta_e_alpha = -P.C_D_delta_e * cos(alpha) + P.C_L_delta_e * sin(alpha);
    C_Z_alpha = - P.C_D_alpha * sin(alpha) - P.C_L_alpha * cos(alpha);
    C_Z_q_alpha = --P.C_D_q * sin(alpha) - P.C_L_q *cos(alpha);
    C_Z_delta_e_alpha = -P.C_D_delta_e * sin(alpha) - P.C_L_delta_e * cos(alpha);
    
    f_ax = 1/2*P.rho*Va^2*P.S_wing * (C_X_alpha  +  C_X_q_alpha * P.c * q/(2*Va) ...
               +  C_X_delta_e_alpha * delta_e);
    f_ay = 1/2*P.rho*Va^2*P.S_wing * (P.C_Y_0  +  P.C_Y_beta * beta  +  P.C_Y_p * P.b * p/(2*Va) ...
               +P.C_Y_r * P.b * r/(2*Va)  +  P.C_Y_delta_a * delta_a  +  P.C_Y_delta_r * delta_r);
    f_az = 1/2*P.rho*Va^2*P.S_wing * (C_Z_alpha  + C_Z_q_alpha * P.c * q/(2*Va)  ...
               +  C_Z_delta_e_alpha * delta_e);          
  %3. Propell Thrust in body frame
    f_px = 1/2*P.rho*P.S_prop*P.C_prop * ((P.k_motor * delta_t)^2-Va^2);
    f_py = 0;
    f_pz = 0;
    
  %4 Ducted fan engine 1~4 gamma 
  %%四个发动机的
    f_Fx = (F1 + F2 + F3 + F4) * cos(gamma);
    f_Fy = 0;
    f_Fz = (F1 + F2 + F3 + F4) * sin(gamma);
    
  %total forces = gravity + aerodynamic + Propell Thrust  
    Force(1) =  f_gx + f_ax + f_px + f_Fx;
    Force(2) =  f_gy + f_ay + f_py + f_Fy; 
    Force(3) =  f_gz + f_az + f_pz + f_Fz;
%---------------------------------------
  %1. Aerodynamic Torque in body frame
    ell = 1/2*P.rho*Va^2*P.S_wing * (P.b*(P.C_ell_0 + P.C_ell_beta * beta + P.C_ell_p*P.b*p/(2*Va) ...
              + P.C_ell_r * P.b *r/(2*Va) + P.C_ell_delta_a * delta_a  +  P.C_Y_delta_r * delta_r));
    m = 1/2*P.rho*Va^2*P.S_wing * (P.c*(P.C_m_0  +  P.C_m_alpha * alpha  +  P.C_m_q* P.c *q/(2*Va) ...
              +  P.C_m_delta_e * delta_e));
    n = 1/2*P.rho*Va^2*P.S_wing * (P.b*(P.C_n_0  +  P.C_n_beta * beta  +  P.C_n_p*P.b*p/(2*Va) ...
              +  P.C_n_r * P.b * r/(2*Va)  +  P.C_n_delta_a * delta_a  +  P.C_n_delta_r * delta_r));
  %2. Propell Torque 
    m_pl = - P.k_T_P * (P.k_Omega * delta_t)^2 ;
    m_pm = 0;
    m_pn = 0;
 
  %3. Ducted fan engine Torque 
    Torque_F_roll = (F2+F3-F1-F4) * P.D * cos(gamma) + P.c_Q/P.c_F * (F1+F3-F2-F4)*sin(gamma);
    Torque_F_picth = P.L * sqrt((1+cos(2*gamma))^2+(sin(2*gamma))^2) * (F1+F2-(F3+F4)...
                                        *sin(atan(1+cos(2*gamma)/sin(2*gamma) - gamma)));
    Torque_F_yaw  = (F2+F3-F1-F4) * P.D * sin(gamma) + P.c_Q/P.c_F * (F1+F3-F2-F4)*cos(gamma);
  %total Torque = aerodynamic Torque + Propell Torque    
    Torque(1) = ell + m_pl + Torque_F_roll;
    Torque(2) = m   + m_pm + Torque_F_picth;   
    Torque(3) = n   + m_pn + Torque_F_yaw;

    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



