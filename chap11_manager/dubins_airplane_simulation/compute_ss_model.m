function [A_lon,B_lon,A_lat,B_lat] = compute_ss_model(filename,x_trim,u_trim)
  % x_trim is the trimmed state,
  % u_trim is the trimmed input
  
  
  [A,B,C,D]=linmod(filename,x_trim,u_trim);


  % extract the lateral state space equations
  E1_lat = [...
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;...
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;...
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;...
    ];
  E2_lat = [...
    0, 1, 0, 0;...
    0, 0, 1, 0;...
    ];
  A_lat = E1_lat*A*E1_lat';
  B_lat = E1_lat*B*E2_lat';

  % extract the longitudinal state space equations 
  E1_lon = [...
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;...
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;...
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;...
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
    ];
  E2_lon = [...
    1, 0, 0, 0;...
    0, 0, 0, 1;...
    ];
  A_lon = E1_lon*A*E1_lon';
  B_lon = E1_lon*B*E2_lon';

  
  