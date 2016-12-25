function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R)
% Va is the desired airspeed (m/s) - Va是期望的空速m/s
% gamma is the desired flight path angle (radians) - gamma是期望的轨迹角
% R is the desired radius (m) - use (+) for right handed orbit,  -R是期望的轨道半径
%                                   (-) for left handed orbit


% add stuff here

x0 = [...
    
    ];

% specify which states to hold equal to the initial conditions - 指定保持等于初始条件的状态
ix = [];

% specify initial inputs  指定的初始输入
u0 = [...
	0;...	% 1 - delta_e - 升降舵量
	0;...	% 2 - delta_a - 横滚舵量
	0;...	% 3 - delta_r - 偏航舵量
	1;...	% 4 - delta_t - 发动机推力量
	];
% specify which inputs to hold constant - 指定保持不变的输入
iu = [];

% define constant outputs - 定义恒定输出
y0 = [...
	Va;...	% 1 - Va - 空速
	0;...	% 2 - alpha - 攻角  
	0;...	% 3 - beta - 侧滑角
	];

% specify which outputs to hold constants - 指定保持不变的输出
iy = [1,3];


% define constant derivatives - 定义常数导数（定义的状态）
dx0 = [...
    0;...               % 1 - pndot
    0;...               % 2 - pedot
    -Va*sin(gamma);...  % 3 - pddot
    0;...               % 4 - udot
    0;...               % 5 - vdot
    0;...               % 6 - wdot
    0;...               % 7 - phidot
    0;...               % 8 - thetadot
    0;...               % 9 - psidot
    0;...               % 10 - pdot
    0;...               % 11 - qdot
    0;...               % 12 - rdot
    ];

if R~=Inf, dx0(9) = Va/R; end  %9 - psidot
% specify which derivaties to hold constant in trin algorithm
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% compute trim conditions
[x_trim,u_trim,y_trim,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);

% check to make sure that the linearization worked (should be small)
norm(dx_trim(3:end)-dx0(3:end))

