% geolocation
%
% compute location of target given position in camera
% input is 
%    uu(1:3)   - camera data (eps_x, eps_y, eps_s)
%    uu(4:15)  - MAV states
%    uu(16:17) - gimbal azimuth, elevation 
%    uu(18)    - time
%
% output is 
%    tn - estimated North postion of target
%    te - estimated East position of target
%    L  - range to target
%
% modified 05/06/2010 - Randy Beard
%
function out = geolocation(in,P)
  
    % process inputs
    NN = 0;
    eps_x     = in(1+NN); % x-pixel
    eps_y     = in(2+NN); % y-pixel
    eps_s     = in(3+NN); % pixel size
    NN = NN + 3;
    pn        = in(1+NN);
    pe        = in(2+NN);
    pd        = -in(3+NN);
    % Va      = in(4+NN);
    % alpha   = in(5+NN);
    % beta    = in(6+NN);
    phi       = in(7+NN);
    theta     = in(8+NN);
    chi       = in(9+NN);
    % p       = in(10+NN);
    % q       = in(11+NN);
    % r       = in(12+NN);
    Vg        = in(13+NN);
    % wn      = in(14+NN);
    % we      = in(15+NN);
    psi     = in(16+NN);
    NN = NN + 16;
    az        = in(1+NN); % gimbal azimuth angle
    el        = in(2+NN); % gimbal elevation angle
    NN = NN + 2;
    t         = in(1+NN); % time

    
    %--------------------------------------
    % begin geolocation code 
    
    
    
    tn   =;
    te   = ;
    L    = ;
    
    
    % end geolocation code 
    %--------------------------------------
    
    % create output
    out = [tn; te; L];
end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_v_to_b(phi,theta,psi)
    % Rotation matrix from body coordinates to vehicle coordinates

    Rot_v_to_v1 = [...
        cos(psi), sin(psi), 0;...
        -sin(psi), cos(psi), 0;...
        0, 0, 1;...
        ];
    
    Rot_v1_to_v2 = [...
        cos(theta), 0, -sin(theta);...
        0, 1, 0;...
        sin(theta), 0, cos(theta);...
        ];
    
    Rot_v2_to_b = [...
        1, 0, 0;...
        0, cos(phi), sin(phi);...
        0, -sin(phi), cos(phi);...
        ];
    
    R = Rot_v2_to_b * Rot_v1_to_v2 * Rot_v_to_v1;

end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_b_to_g(az,el)
    % Rotation matrix from body coordinates to gimbal coordinates
    Rot_b_to_g1 = [...
        cos(az), sin(az), 0;...
        -sin(az), cos(az), 0;...
        0, 0, 1;...
        ];

    Rot_g1_to_g = [...
        cos(el), 0, -sin(el);...
        0, 1, 0;...
        sin(el), 0, cos(el);...
        ];

    R = Rot_g1_to_g * Rot_b_to_g1;

end
