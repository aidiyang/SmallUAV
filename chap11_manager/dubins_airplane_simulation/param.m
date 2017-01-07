P.gravity = 9.8;
   
%physical parameters of airframe
P.mass = 1.56;
P.Jx   = 0.1147;
P.Jy   = 0.0576;
P.Jz   = 0.1712;
P.Jxz  = 0.0015;

% aerodynamic coefficients
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
P.rho           = 1.2682;
P.c             = 0.3302;
P.b             = 1.4224;
P.S_wing        = 0.2589;
P.S_prop        = 0.0314;
P.k_motor       = 20;
P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_M_0         = 0.0;
P.C_M_alpha     = -0.38;
P.C_M_q         = -3.6;
P.C_M_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = -0.26;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1;


% wind parameters
P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;
P.L_wx = 1250;
P.L_wy = 1750;
P.L_wz = 1750;
P.sigma_wx = 1; 
P.sigma_wy = 1;
P.sigma_wz = 0.7;
% P.sigma_wx = 0; 
% P.sigma_wy = 0;
% P.sigma_wz = 0;
P.Va0 = 13;

% autopilot sample rate
P.Ts = 0.01;

% compute trim conditions using 'mavsim_chap5_trim.mdl'
P.Va    = 13;         % desired airspeed (m/s)
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = 0;         % desired radius (m) - use (+) for right handed orbit, 
                    %                          (-) for left handed orbit
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -100;  % initial Down position (negative altitude)
P.u0     = P.Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0*pi/180;  % initial heading angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
   = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% gain on dirty derivative
P.tau = 5;

% autopilot gains
% altitude parameters and gains
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 10;
computeGains;

% sensor parameters
%P.sigma_gyro = 0.13*pi/180; % standard deviation of gyros in rad/sec
%P.bias_gyro_x = 0.1*pi/180*rand; % bias on x_gyro
%P.bias_gyro_y = 0.1*pi/180*rand; % bias on y_gyro
%P.bias_gyro_z = 0.1*pi/180*rand; % bias on z_gyro
%P.sigma_accel = 0.0025*9.8; % standard deviation of accelerometers in m/s^2
%P.sigma_static_pres = 0.01*1000; % standard deviation of static pressure sensor in Pascals
%P.sigma_diff_pres = 0.002*1000;  % standard deviation of diff pressure sensor in Pascals

% set sensor noise to zero to test KF with no noise (should work great)
% P.sigma_gyro = 0.0; % standard deviation of gyros in rad/sec
% P.bias_gyro_x = 0.0; % bias on x_gyro
% P.bias_gyro_y = 0.0; % bias on y_gyro
% P.bias_gyro_z = 0.0; % bias on z_gyro
% P.sigma_accel = 0.0; % standard deviation of accelerometers in m/s^2
% P.sigma_static_pres = 0.0; % standard deviation of static pressure sensor in Pascals
% P.sigma_diff_pres = 0.0;  % standard deviation of diff pressure sensor in Pascals


% GPS parameters
%P.Ts_gps = 1; % sample rate of GPS in s
% P.beta_gps = 1/1100; % 1/s
% P.sigma_n_gps = 0.21;
% P.sigma_e_gps = 0.21; 
% P.sigma_h_gps = 0.40;
% P.sigma_Vg_gps = 0.05;
% P.sigma_course_gps = P.sigma_Vg_gps/P.Va;


% parameters for guidance model
zeta_chi = .7;
wn_chi   = 0.75;
% zeta_chi = .7;
% wn_chi   = 2;
P.b_chi    = wn_chi^2;
P.b_chidot = 2*zeta_chi*wn_chi;

P.b_Va     = 1;

zeta_h       = 1.1;
wn_h         = .6;
P.b_h      = wn_h^2;
P.b_hdot   = 2*zeta_h*wn_h;
P.a_hdot   = .01;
P.b_hnum   = 0;

P.b_phi    = 1/0.2;


% number of waypoints in data structure
P.size_waypoint_array = 100;
P.R_min = 40;  % minimum turn radius
P.gam_max = 10*pi/180; % maximum flight path angle

% create random city map
city_width      = 500;  % the city is of size (width)x(width)
building_height = 50;   % maximum height of buildings
%building_height = 1;   % maximum height of buildings (for camera)
num_blocks      = 5;    % number of blocks in city
street_width    = .8;   % percent of block that is street.
%P.pd0           = -30;  % initial height of MAV
map = createWorld(city_width, building_height, num_blocks, street_width);

% % target parameters
% P.target_velocity = 5;  % (m/s)
% P.target_size = 2;          % size of target 
% 
% 
% % gimbal parameters
% P.az0 = 0;      % initial azimuth angle
% P.el0 = -pi/2;  % initial elevation angle (pointing down)
% P.az_limit = 180*(pi/180);  % azimuth angle limit
% P.el_limit = 180*(pi/180);  % elevation angle limit
% P.az_gain  = 1;  % gain on azimuth dynamics (azdot = az_gain*u_az)
% P.el_gain  = 1;  % gain on elevation dynamics (eldot = el_gain*u_el)
% P.k_az     = 10; % proportional control gain for gimbal azimuth
% P.k_el     = 10; % proportional control gain for gimbal elevation
% 
% % camera parameters
% P.cam_fps = 10;  % frames per second 
% P.cam_pix = 480;                      % size of (square) pixel array
% P.cam_fov   = 10*(pi/180);            % field of view of camera
% P.f = (P.cam_pix/2)/tan(P.cam_fov/2); % focal range
% P.pixelnoise = 0;                     % (pixels) - variance of the pixel noise
% 
% % measurement model for GPS (used in geolocation algorithm)
% P.sigma_measurement_n = 0.547; % standard deviation of gps measurement error in m
% P.sigma_measurement_e = 0.547; % standard deviation of gps measurement error in m
% P.sigma_measurement_h = 1.14; % standard deviation of gps measurement error in m
% 


