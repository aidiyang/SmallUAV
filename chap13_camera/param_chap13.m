
% target parameters
P.target_velocity = 5;  % (m/s)
P.target_size = 2;          % size of target 


% gimbal parameters
P.az0 = 0;      % initial azimuth angle
P.el0 = -pi/2;  % initial elevation angle (pointing down)
P.az_limit = 180*(pi/180);  % azimuth angle limit
P.el_limit = 180*(pi/180);  % elevation angle limit
P.az_gain  = 1;  % gain on azimuth dynamics (azdot = az_gain*u_az)
P.el_gain  = 1;  % gain on elevation dynamics (eldot = el_gain*u_el)
P.k_az     = 10; % proportional control gain for gimbal azimuth
P.k_el     = 10; % proportional control gain for gimbal elevation

% camera parameters
P.cam_fps = 10;  % frames per second 
P.cam_pix = 480;                      % size of (square) pixel array
P.cam_fov   = 10*(pi/180);            % field of view of camera
P.f = (P.cam_pix/2)/tan(P.cam_fov/2); % focal range
P.pixelnoise = 0;                     % (pixels) - variance of the pixel noise

% measurement model for GPS (used in geolocation algorithm)
P.sigma_measurement_n = 0.547; % standard deviation of gps measurement error in m
P.sigma_measurement_e = 0.547; % standard deviation of gps measurement error in m
P.sigma_measurement_h = 1.14; % standard deviation of gps measurement error in m



