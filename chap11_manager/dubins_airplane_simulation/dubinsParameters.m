%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dubinsParameters
%   - Find Dubins airplane parameters between two configurations
%
% Modified:  
%   - 4/13/2010 - RWB
%   - 5/8/2013  - RWB
%
% input is:
%   start_node  - [wn_s, we_s, wd_s, chi_s, 0, 0]
%   end_node    - [wn_e, wn_e, wd_e, chi_e, 0, 0]
%   R           - minimum turn radius
%
% output is:
%   dubinspath  - a matlab structure with the following fields
%    dubinspath.case  1==SLS, 2==SSLS, 3==SLSS, 4==SSLSS, where S==spiral, L==line
%   // start and end configurations
%    dubinspath.p_s   - start position
%    dubinspath.chi_s - start angle
%    dubinspath.p_e   - end position
%    dubinspath.chi_e - end angle
%   // general path parameters
%    dubinspath.R    - radius of spirals
%    dubinspath.gam  - flight path angle (constant throughout maneuver)
%    dubinspath.L    - length of Dubins path
%   // start spiral
%    dubinspath.c_s   - center of start spiral
%    dubinspath.psi_s - angle of start spiral
%    dubinspath.lam_s - direction of start spiral
%    dubinspath.k_s   - number of full start spirals
%   // intermediate-start spiral
%    dubinspath.c_si   - center of intermediate-start spiral
%    dubinspath.psi_si - angle of intermediate-start spiral
%    dubinspath.lam_si - direction of intermediate-start spiral
%    dubinspath.k_si   - number of full intermediate-start spirals
%   //intermediate-end spiral
%    dubinspath.c_ei   - center of intermediate-end spiral
%    dubinspath.psi_ei - angle of intermediate-end spiral
%    dubinspath.lam_ei - direction of intermediate-end spiral
%    dubinspath.k_ei   - number of full intermediate-end spirals
%   // end spiral
%    dubinspath.c_e   - center of end spiral
%    dubinspath.psi_e - angle of end spiral
%    dubinspath.lam_e - direction of end spiral
%    dubinspath.k_e   - number of full end spirals
%   // hyperplane H_s: switch from first spiral to next segment
%    dubinspath.w_s   - center of hyperplane
%    dubinspath.q_s   - normal vector to hyperplane
%   // hyperplane H_si: switch from second start spiral to next segment
%    dubinspath.w_si  - center of hyperplane
%    dubinspath.q_si  - normal vector to hyperplane
%   // hyperplane H_l: switch from straight-line to next segment
%    dubinspath.w_l   - center of hyperplane
%    dubinspath.q_l   - normal vector to hyperplane
%   // hyperplane H_ei: switch from second end spiral 
%    dubinspath.w_ei  - center of hyperplane
%    dubinspath.q_ei  - normal vector to hyperplane
%   // hyperplane H_e: end of Dubins path 
%    dubinspath.w_e   - center of hyperplane
%    dubinspath.q_e   - normal vector to hyperplane

function dubinspath = dubinsParameters(start_node, end_node, R_min, gam_max)

%   ell = norm(start_node(1:2)-end_node(1:2));
%   if ell<2*R,
%       disp('The distance between nodes must be larger than 2R.');
%       dubinspath = [];
%   else
    
    zs   = start_node(1:3)';
    chis = start_node(4);
    ze   = end_node(1:3)';
    chie = end_node(4);
    % start and end configurations
    dubinspath.p_s   = zs;   % start position
    dubinspath.chi_s = chis; % start angle
    dubinspath.p_e   = ze;   % end position
    dubinspath.chi_e = chie; % end angle


    crs = zs + R_min*rotz(pi/2) *[cos(chis), sin(chis), 0]';
    cls = zs + R_min*rotz(-pi/2)*[cos(chis), sin(chis), 0]';
    cre = ze + R_min*rotz(pi/2) *[cos(chie), sin(chie), 0]';
    cle = ze + R_min*rotz(-pi/2)*[cos(chie), sin(chie), 0]';
    
    % compute L1, L2, L3, L4
    L1 = computeDubinsRSR(R_min, crs, cre, chis, chie);
    L2 = computeDubinsRSL(R_min, crs, cle, chis, chie);
    L3 = computeDubinsLSR(R_min, cls, cre, chis, chie);
    L4 = computeDubinsLSL(R_min, cls, cle, chis, chie);
    
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    
    hdist = -(ze(3)-zs(3));  % negative is because of down coordinates
    %----low altitude climbs-------
    if abs(hdist)<=L*tan(gam_max), 
        gam = atan(hdist/L);
        dubinspath.case = 1; % the Dubins path is of the form Spiral-Line-Spiral
        dubinspath.R    = R_min;    % radius of spirals
        dubinspath.gam  = gam;  % flight path angle (constant throughout maneuver)
        dubinspath.L    = L/cos(gam);    % length of Dubins path
        dubinspath.k_s  = 0;   
        dubinspath.k_e  = 0;
        
        L

    %----high altitude climbs-------
    elseif abs(hdist)>=(L+2*pi*R_min)*tan(gam_max), 
        % find number of required spirals
        k=floor((abs(hdist)/tan(gam_max)-L)/(2*pi*R_min));
        if hdist>=0, % end at higher altitude than start: spiral at start
            dubinspath.k_s  = k;   
            dubinspath.k_e  = 0;
        else % spiral at end
            dubinspath.k_s  = 0;   
            dubinspath.k_e  = k;
        end
        % find optimal turning radius
        R = computeOptimalRadius(zs, chis, ze, chie, R_min, gam_max, idx, k, hdist);
        % recompute centers of spirals and Dubins path length with new R
        crs = zs + R*rotz(pi/2) *[cos(chis), sin(chis), 0]';
        cls = zs + R*rotz(-pi/2)*[cos(chis), sin(chis), 0]';
        cre = ze + R*rotz(pi/2) *[cos(chie), sin(chie), 0]';
        cle = ze + R*rotz(-pi/2)*[cos(chie), sin(chie), 0]';
        switch(idx),
            case 1, L = computeDubinsRSR(R, crs, cre, chis, chie);
            case 2, L = computeDubinsRSL(R, crs, cle, chis, chie);
            case 3, L = computeDubinsLSR(R, cls, cre, chis, chie);
            case 4, L = computeDubinsLSL(R, cls, cle, chis, chie);
        end
        dubinspath.case = 1; % the Dubins path is of the form Spiral-Line-Spiral
        dubinspath.R    = R;    % radius of spirals
        gam             = sign(hdist)*gam_max; % flight path angle is equal to limit
        dubinspath.gam  = gam;
        dubinspath.L    = (L+2*pi*k*R)/cos(gam_max);    % length of Dubins path
               
    %----medium altitude climbs-------
    else 
        % flight path angle is equal to limit
        gam = sign(hdist)*gam_max;
        if hdist>0, % add third spiral at beginning
            [zi, chii, L, ci, psii] = addSpiralBeginning(zs, chis, ze, chie, R_min, gam, idx, hdist);
            dubinspath.case = 2; % Dubins path is of the form Spiral-Spiral-Line-Spiral
        else % add third spiral at end
            [zi, chii, L, ci, psii] = addSpiralEnd(zs, chis, ze, chie, R_min, gam, idx, hdist); 
            dubinspath.case = 3; % Dubins path is of the form Spiral-Line-Spiral-Spiral
        end
        dubinspath.R    = R_min; % minimum turn radius   
        dubinspath.gam  = gam;  % flight path angle is equal to limit
        dubinspath.L    = L/cos(gam_max); % length of Dubins path
    end
    
    e1 = [1; 0; 0];
    R = dubinspath.R;
    switch(dubinspath.case),
        case 1, % spiral - line - spiral
            switch(idx),
                case 1, % right - straight - right
                    theta = atan2(cre(2)-crs(2),cre(1)-crs(1));
                    dist1 = R*mod(2*pi+mod(theta-pi/2,2*pi)-mod(chis-pi/2,2*pi),2*pi)...
                        + 2*pi*R*dubinspath.k_s;
                    dist2 = R*mod(2*pi+mod(chie-pi/2,2*pi)-mod(theta-pi/2,2*pi),2*pi)...
                        + 2*pi*R*dubinspath.k_e;
                    w1 = crs + dubinspath.R*rotz(theta-pi/2)*e1+[0;0;-dist1*tan(gam)];
                    w2 = cre + dubinspath.R*rotz(theta-pi/2)*e1-[0;0;-dist2*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1); % direction of line
                    % start spiral
                    dubinspath.c_s   = crs;       
                    dubinspath.psi_s = chis-pi/2; 
                    dubinspath.lam_s = 1;         
                    % end spiral
                    dubinspath.c_e   = cre-[0;0;-dist2*tan(gam)];       
                    dubinspath.psi_e = theta-pi/2; 
                    dubinspath.lam_e = 1;         
                    % hyperplane H_s: switch from first spiral to line
                    dubinspath.w_s   = w1;
                    dubinspath.q_s   = q1;
                    % hyperplane H_l: switch from line to last spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];
                    
                case 2, % right - straight - left
                    ell = norm(cle(1:2)-crs(1:2));
                    theta = atan2(cle(2)-crs(2),cle(1)-crs(1));
                    theta2 = theta-pi/2+asin(2*R/ell);
                    dist1 = R*mod(2*pi+mod(theta2,2*pi)-mod(chis-pi/2,2*pi),2*pi)...
                        + 2*pi*R*dubinspath.k_s;
                    dist2 = R*mod(2*pi+mod(theta2+pi,2*pi)-mod(chie+pi/2,2*pi),2*pi)...
                        + 2*pi*R*dubinspath.k_e;
                    w1 = crs + R*rotz(theta2)*e1 + [0; 0; -dist1*tan(gam)];
                    w2 = cle + R*rotz(theta2+pi)*e1-[0; 0; -dist2*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1);
                    % start spiral
                    dubinspath.c_s   = crs;       
                    dubinspath.psi_s = chis-pi/2; 
                    dubinspath.lam_s = 1;         
                    % end spiral
                    dubinspath.c_e   = cle-[0;0;-dist2*tan(gam)];       
                    dubinspath.psi_e = theta2+pi; 
                    dubinspath.lam_e = -1;         
                    % hyperplane H_s: switch from first spiral to line
                    dubinspath.w_s   = w1;
                    dubinspath.q_s   = q1;
                    % hyperplane H_l: switch from line to end spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];
                    
                case 3, % LSR
                    ell = norm(cre(1:2)-cls(1:2));
                    theta = atan2(cre(2)-cls(2),cre(1)-cls(1));
                    theta2 = acos(2*R/ell);
                    dist1 = R*mod(2*pi-mod(theta+theta2,2*pi) + mod(chis+pi/2,2*pi),2*pi)...
                        + 2*pi*R*dubinspath.k_s;
                    dist2 = R*mod(2*pi-mod(theta+theta2-pi,2*pi)+mod(chie-pi/2,2*pi),2*pi)...
                        + 2*pi*R*dubinspath.k_e;
                    w1 = cls + R*rotz(theta+theta2)*e1 + [0; 0; -dist1*tan(gam)];
                    w2 = cre + R*rotz(-pi+theta+theta2)*e1 - [0; 0; -dist2*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1);
                    % start spiral
                    dubinspath.c_s   = cls;       
                    dubinspath.psi_s = chis+pi/2; 
                    dubinspath.lam_s = -1;         
                    % end spiral
                    dubinspath.c_e   = cre-[0;0;-dist2*tan(gam)];       
                    dubinspath.psi_e = mod(theta+theta2-pi,2*pi); 
                    dubinspath.lam_e = 1;         
                    % hyperplane H_s: switch from first spiral to line
                    dubinspath.w_s   = w1;
                    dubinspath.q_s   = q1;
                    % hyperplane H_l: switch from line to end spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1; 0; 0];
                    
                case 4, % left  - straight - left
                    theta = atan2(cle(2)-cls(2),cle(1)-cls(1));
                    dist1 = R*mod(2*pi-mod(theta+pi/2,2*pi)+mod(chis+pi/2,2*pi),2*pi)...
                        + 2*pi*R*dubinspath.k_s;
                    dist2 = R*mod(2*pi-mod(chie+pi/2,2*pi)+mod(theta+pi/2,2*pi),2*pi)...
                        + 2*pi*R*dubinspath.k_e;
                    w1 = cls + dubinspath.R*rotz(theta+pi/2)*e1+[0;0;-dist1*tan(gam)];
                    w2 = cle + dubinspath.R*rotz(theta+pi/2)*e1-[0;0;-dist2*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1); % direction of line
                    % start spiral
                    dubinspath.c_s   = cls;       
                    dubinspath.psi_s = chis+pi/2; 
                    dubinspath.lam_s = -1;         
                    % end spiral
                    dubinspath.c_e   = cle-[0;0;-dist2*tan(gam)];       
                    dubinspath.psi_e = theta+pi/2; 
                    dubinspath.lam_e = -1;         
                    % hyperplane H_s: switch from first spiral to line
                    dubinspath.w_s   = w1;
                    dubinspath.q_s   = q1;
                    % hyperplane H_l: switch from line to end spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];                   
            end
        case 2, % spiral - spiral - line - spiral
            switch(idx),
                case 1, % right - left- straight - right
                    % start spiral
                    dubinspath.c_s   = crs;       
                    dubinspath.psi_s = chis-pi/2; 
                    dubinspath.lam_s = 1;         
                    dubinspath.k_s   = 0;                     
                    ell = norm(cre(1:2)-ci(1:2));
                    theta = atan2(cre(2)-ci(2),cre(1)-ci(1));
                    theta2 = acos(2*R/ell);
                    dist1 = R_min*psii + R*mod(2*pi-mod(theta+theta2,2*pi) + mod(chii+pi/2,2*pi),2*pi);
                    dist2 = R*mod(2*pi-mod(theta+theta2-pi,2*pi)+mod(chie-pi/2,2*pi),2*pi);
                    w1 = ci + R*rotz(theta+theta2)*e1 + [0; 0; -dist1*tan(gam)];
                    w2 = cre + R*rotz(-pi+theta+theta2)*e1 - [0; 0; -dist2*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1);
                    % intermediate-start spiral
                    dubinspath.c_si   = ci + [0; 0; -R_min*psii*tan(gam)];       
                    dubinspath.psi_si = chii+pi/2;       
                    dubinspath.lam_si = -1;       
                    dubinspath.k_si   = 0;       
                    % end spiral
                    dubinspath.c_e   = cre-[0;0;-dist2*tan(gam)];       
                    dubinspath.psi_e = mod(theta+theta2-pi,2*pi); 
                    dubinspath.lam_e = 1;         
                    dubinspath.k_e   = 0;        
                    % hyperplane H_s: switch from first to second spiral
                    dubinspath.w_s   = zi-[0;0;-psii*R_min*tan(gam)];
                    dubinspath.q_s   = [cos(chii); sin(chii); 0];
                    % hyperplane H_si: switch from second spiral to straight line
                    dubinspath.w_si  = w1;
                    dubinspath.q_si  = q1;
                    % hyperplane H_l: switch from straight-line to end spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];
                    
                case 2, % right - left- straight - left
                    theta = atan2(cle(2)-ci(2),cle(1)-ci(1));
                    dist1 = R*mod(2*pi-mod(theta+pi/2,2*pi)+mod(chii+pi/2,2*pi),2*pi);
                    dist2 = psii*R;
                    dist3 = R*mod(2*pi-mod(chie+pi/2,2*pi)+mod(theta+pi/2,2*pi),2*pi);
                    w1 = ci + dubinspath.R*rotz(theta+pi/2)*e1+[0;0;-(dist1+dist2)*tan(gam)];
                    w2 = cle + dubinspath.R*rotz(theta+pi/2)*e1-[0;0;-dist3*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1); % direction of line
                    % start spiral
                    dubinspath.c_s   = crs;       
                    dubinspath.psi_s = chis-pi/2; 
                    dubinspath.lam_s = 1;         
                    dubinspath.k_s   = 0; 
                    % intermediate-start spiral
                    dubinspath.c_si   = ci + [0; 0; -dist2*tan(gam)];       
                    dubinspath.psi_si = chii+pi/2;       
                    dubinspath.lam_si = -1;       
                    dubinspath.k_si   = 0;       
                    % end spiral
                    dubinspath.c_e   = cle-[0;0;-dist3*tan(gam)];       
                    dubinspath.psi_e = theta+pi/2; 
                    dubinspath.lam_e = -1;         
                    dubinspath.k_e   = 0;        
                    % hyperplane H_s: switch from first to second spiral
                    dubinspath.w_s   = zi-[0;0;-dist2*tan(gam)];
                    dubinspath.q_s   = [cos(chii); sin(chii); 0];
                    % hyperplane H_si: switch from second spiral to straight line
                    dubinspath.w_si  = w1;
                    dubinspath.q_si  = q1;
                    % hyperplane H_l: switch from straight-line to end spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];
                    
                case 3, % LRSR 
                    theta = atan2(cre(2)-ci(2),cre(1)-ci(1));
                    dist1 = R*mod(2*pi+mod(theta-pi/2,2*pi)-mod(chii-pi/2,2*pi),2*pi);
                    dist2 = psii*R;
                    dist3 = R*mod(2*pi+mod(chie-pi/2,2*pi)-mod(theta-pi/2,2*pi),2*pi);
                    w1 = ci + dubinspath.R*rotz(theta-pi/2)*e1+[0;0;-(dist1+dist2)*tan(gam)];
                    w2 = cre + dubinspath.R*rotz(theta-pi/2)*e1-[0;0;-dist3*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1); % direction of line
                    % start spiral
                    dubinspath.c_s   = cls;       
                    dubinspath.psi_s = chis+pi/2; 
                    dubinspath.lam_s = -1;         
                    dubinspath.k_s   = 0;                     
                    % intermediate-start spiral
                    dubinspath.c_si   = ci + [0; 0; -dist2*tan(gam)];       
                    dubinspath.psi_si = chii-pi/2;       
                    dubinspath.lam_si = 1;       
                    dubinspath.k_si   = 0;       
                    % end spiral
                    dubinspath.c_e   = cre-[0;0;-dist3*tan(gam)];       
                    dubinspath.psi_e = theta-pi/2; 
                    dubinspath.lam_e = 1;         
                    dubinspath.k_e   = 0;        
                    % hyperplane H_s: switch from first to second spiral
                    dubinspath.w_s   = zi-[0;0;-dist2*tan(gam)];
                    dubinspath.q_s   = [cos(chii); sin(chii); 0];
                    % hyperplane H_si: switch from second spiral to straight line
                    dubinspath.w_si  = w1;
                    dubinspath.q_si  = q1;
                    % hyperplane H_l: switch from straight-line to end spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];
                    
                case 4, % LRSL
                    ell = norm(cle(1:2)-ci(1:2));
                    theta = atan2(cle(2)-ci(2),cle(1)-ci(1));
                    theta2 = theta-pi/2+asin(2*R/ell);
                    dist1 = R*mod(2*pi+mod(theta2,2*pi)-mod(chii-pi/2,2*pi),2*pi);
                    dist2 = R*psii;
                    dist3 = R*mod(2*pi+mod(theta2+pi,2*pi)-mod(chie+pi/2,2*pi),2*pi);
                    w1 = ci + R*rotz(theta2)*e1 + [0; 0; -(dist1+dist2)*tan(gam)];
                    w2 = cle + R*rotz(theta2+pi)*e1-[0; 0; -dist3*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1);
                    % start spiral
                    dubinspath.c_s   = cls;       
                    dubinspath.psi_s = chis+pi/2; 
                    dubinspath.lam_s = -1;         
                    dubinspath.k_s   = 0;                     
                    % intermediate-start spiral
                    dubinspath.c_si   = ci + [0; 0; -dist2*tan(gam)];       
                    dubinspath.psi_si = chii-pi/2;       
                    dubinspath.lam_si = 1;       
                    dubinspath.k_si   = 0;       
                    % end spiral
                    dubinspath.c_e   = cle-[0;0;-dist3*tan(gam)];       
                    dubinspath.psi_e = theta2+pi; 
                    dubinspath.lam_e = -1;         
                    dubinspath.k_e   = 0;        
                    % hyperplane H_s: switch from first to second spiral
                    dubinspath.w_s   = zi-[0;0;-dist2*tan(gam)];
                    dubinspath.q_s   = [cos(chii); sin(chii); 0];
                    % hyperplane H_si: switch from second spiral to straight line
                    dubinspath.w_si  = w1;
                    dubinspath.q_si  = q1;
                    % hyperplane H_l: switch from straight-line to end spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];                    
            end
            
        case 3, % spiral - line - spiral - spiral
            switch(idx),
                case 1, % RSLR
                    % path specific calculations
                    ell = norm(ci(1:2)-crs(1:2));
                    theta = atan2(ci(2)-crs(2),ci(1)-crs(1));
                    theta2 = theta-pi/2+asin(2*R/ell);
                    dist1 = R*mod(2*pi+mod(theta2,2*pi)-mod(chis-pi/2,2*pi),2*pi);
                    dist2 = R*mod(2*pi+mod(theta2+pi,2*pi)-mod(chii+pi/2,2*pi),2*pi);
                    dist3 = abs(R_min*psii);
                    w1 = crs + R*rotz(theta2)*e1 + [0; 0; -dist1*tan(gam)];
                    w2 = ci + R*rotz(theta2+pi)*e1 - [0; 0; -(dist2+dist3)*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1);
                    % start spiral
                    dubinspath.c_s   = crs;       
                    dubinspath.psi_s = chis-pi/2; 
                    dubinspath.lam_s = 1;         
                    dubinspath.k_s   = 0;                           
                    % intermediate-end spiral
                    dubinspath.c_ei   = ci - [0; 0; -(dist2+dist3)*tan(gam)];       
                    dubinspath.psi_ei = theta2+pi;       
                    dubinspath.lam_ei = -1;       
                    dubinspath.k_ei   = 0;       
                    % end spiral
                    dubinspath.c_e   = cre - [0;0;-dist3*tan(gam)];       
                    dubinspath.psi_e = -psii; 
                    dubinspath.lam_e = 1;         
                    dubinspath.k_e   = 0;        
                    % hyperplane H_s: switch from first to second spiral
                    dubinspath.w_s   = w1;
                    dubinspath.q_s   = q1;
                    % hyperplane H_l: switch from straight-line to intermediate spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_ei: switch from intermediate spiral to
                    % end spiral
                    dubinspath.w_ei  = zi - [0;0;-dist3*tan(gam)];
                    dubinspath.q_ei  = [cos(chii); sin(chii); 0];
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];

                case 2, % RSRL
                    % path specific calculations
                    theta = atan2(ci(2)-crs(2),ci(1)-crs(1));
                    dist1 = R*mod(2*pi+mod(theta-pi/2,2*pi)-mod(chis-pi/2,2*pi),2*pi);
                    dist2 = R*mod(2*pi+mod(chii-pi/2,2*pi)-mod(theta-pi/2,2*pi),2*pi);
                    dist3 = abs(R_min*psii);
                    w1 = crs + R*rotz(theta-pi/2)*e1+[0;0;-dist1*tan(gam)];
                    w2 = ci  + R*rotz(theta-pi/2)*e1-[0;0;-(dist2+dist3)*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1);
                    % start spiral
                    dubinspath.c_s   = crs;       
                    dubinspath.psi_s = chis-pi/2; 
                    dubinspath.lam_s = 1;         
                    dubinspath.k_s   = 0;                           
                    % intermediate-end spiral
                    dubinspath.c_ei   = ci - [0; 0; -(dist2+dist3)*tan(gam)];       
                    dubinspath.psi_ei = theta-pi/2;       
                    dubinspath.lam_ei = 1;       
                    dubinspath.k_ei   = 0;       
                    % end spiral
                    dubinspath.c_e   = cle - [0;0;-dist3*tan(gam)];       
                    dubinspath.psi_e = psii; 
                    dubinspath.lam_e = -1;         
                    dubinspath.k_e   = 0;        
                    % hyperplane H_s: switch from first to second spiral
                    dubinspath.w_s   = w1;
                    dubinspath.q_s   = q1;
                    % hyperplane H_l: switch from straight-line to intermediate spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_ei: switch from intermediate spiral to
                    % end spiral
                    dubinspath.w_ei  = zi - [0;0;-dist3*tan(gam)];
                    dubinspath.q_ei  = [cos(chii); sin(chii); 0];
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];
                     
                case 3, % LSLR
                    % path specific calculations
                    theta = atan2(ci(2)-cls(2),ci(1)-cls(1));
                    dist1 = R*mod(2*pi-mod(theta+pi/2,2*pi)+mod(chis+pi/2,2*pi),2*pi);
                    dist2 = R*mod(2*pi-mod(chii+pi/2,2*pi)+mod(theta+pi/2,2*pi),2*pi);
                    dist3 = abs(R_min*psii);
                    w1 = cls + dubinspath.R*rotz(theta+pi/2)*e1+[0;0;-dist1*tan(gam)];
                    w2 = ci + dubinspath.R*rotz(theta+pi/2)*e1-[0;0;-(dist2+dist3)*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1); % direction of line
                    % start spiral
                    dubinspath.c_s   = cls;       
                    dubinspath.psi_s = chis+pi/2; 
                    dubinspath.lam_s = -1;         
                    dubinspath.k_s   = 0;                           
                    % intermediate-end spiral
                    dubinspath.c_ei   = ci - [0; 0; -(dist2+dist3)*tan(gam)];       
                    dubinspath.psi_ei = theta+pi/2;       
                    dubinspath.lam_ei = -1;       
                    dubinspath.k_ei   = 0;       
                    % end spiral
                    dubinspath.c_e   = cre - [0;0;-dist3*tan(gam)];       
                    dubinspath.psi_e = -psii; 
                    dubinspath.lam_e = 1;         
                    dubinspath.k_e   = 0;        
                    % hyperplane H_s: switch from first to second spiral
                    dubinspath.w_s   = w1;
                    dubinspath.q_s   = q1;
                    % hyperplane H_l: switch from straight-line to intermediate spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_ei: switch from intermediate spiral to
                    % end spiral
                    dubinspath.w_ei  = zi - [0;0;-dist3*tan(gam)];
                    dubinspath.q_ei  = [cos(chii); sin(chii); 0];
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];

                case 4, % LSRL
                    % path specific calculations
                    ell = norm(ci(1:2)-cls(1:2));
                    theta = atan2(ci(2)-cls(2),ci(1)-cls(1));
                    theta2 = acos(2*R/ell);
                    dist1 = R*mod(2*pi-mod(theta+theta2,2*pi) + mod(chis+pi/2,2*pi),2*pi);
                    dist2 = R*mod(2*pi-mod(theta+theta2-pi,2*pi)+mod(chii-pi/2,2*pi),2*pi);
                    dist3 = abs(R_min*psii);
                    w1 = cls + R*rotz(theta+theta2)*e1 + [0; 0; -dist1*tan(gam)];
                    w2 = ci + R*rotz(-pi+theta+theta2)*e1 - [0; 0; -(dist2+dist3)*tan(gam)];
                    q1 = (w2-w1)/norm(w2-w1);
                    % start spiral
                    dubinspath.c_s   = cls;       
                    dubinspath.psi_s = chis+pi/2; 
                    dubinspath.lam_s = -1;         
                    dubinspath.k_s   = 0;                           
                    % intermediate-end spiral
                    dubinspath.c_ei   = ci - [0; 0; -(dist2+dist3)*tan(gam)];       
                    dubinspath.psi_ei = mod(theta+theta2-pi,2*pi);       
                    dubinspath.lam_ei = 1;       
                    dubinspath.k_ei   = 0;       
                    % end spiral
                    dubinspath.c_e   = cle - [0;0;-dist3*tan(gam)];       
                    dubinspath.psi_e = psii; 
                    dubinspath.lam_e = -1;         
                    dubinspath.k_e   = 0;        
                    % hyperplane H_s: switch from first to second spiral
                    dubinspath.w_s   = w1;
                    dubinspath.q_s   = q1;
                    % hyperplane H_l: switch from straight-line to intermediate spiral
                    dubinspath.w_l   = w2;
                    dubinspath.q_l   = q1;
                    % hyperplane H_ei: switch from intermediate spiral to
                    % end spiral
                    dubinspath.w_ei  = zi - [0;0;-dist3*tan(gam)];
                    dubinspath.q_ei  = [cos(chii); sin(chii); 0];
                    % hyperplane H_e: end of Dubins path 
                    dubinspath.w_e   = ze;
                    dubinspath.q_e   = rotz(chie)*[1;0;0];
             end
        
        case 4, % spiral - spiral - line - spiral - spiral
            disp('This case is not implemented')
    end
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rotz(theta)
%   rotation matrix about the z axis.
function R = rotz(theta)
    R = [...
        cos(theta), -sin(theta), 0;...
        sin(theta), cos(theta), 0;...
        0, 0, 1;...
        ];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roty(theta)
%   rotation matrix about the y axis.
function R = roty(theta)
    R = [...
        cos(theta), 0, sin(theta);...
        0, 1, 0;...
        -sin(theta), 0, cos(theta);...
        ];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% L = computeDubinsRSR(R, crs, cre, chis, chie)
%   Find the length of Right-Straight-Right Dubins car path.
%   Inputs:
%      R    - turn radius
%      crs  - center of right start circle
%      cre  - center of right end cirle
%      chis - start heading
%      chie - end heading
%    Ouputs:
%      L    - length of path
function L = computeDubinsRSR(R, crs, cre, chis, chie)
    theta = atan2(cre(2)-crs(2),cre(1)-crs(1));
    L = norm(crs(1:2)-cre(1:2))...
        + R*mod(2*pi+mod(theta-pi/2,2*pi)-mod(chis-pi/2,2*pi),2*pi)...
        + R*mod(2*pi+mod(chie-pi/2,2*pi)-mod(theta-pi/2,2*pi),2*pi);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% L = computeDubinsRSL(R, crs, cle, chis, chie)
%   Find the length of Right-Straight-Left Dubins car path.
%   Inputs:
%      R    - turn radius
%      crs  - center of right start circle
%      cle  - center of left end cirle
%      chis - start heading
%      chie - end heading
%    Ouputs:
%      L    - length of path
function L = computeDubinsRSL(R, crs, cle, chis, chie)
    ell = norm(cle(1:2)-crs(1:2));
    theta = atan2(cle(2)-crs(2),cle(1)-crs(1));
    theta2 = theta-pi/2+asin(2*R/ell);
    if isreal(theta2)==0, 
        L = 99999999; 
    else
% Test typo fix in equation 11.10        
%       L = sqrt(ell^2-4*R^2) + R*mod(2*pi+mod(theta-theta2,2*pi)-mod(chis-pi/2,2*pi),2*pi)...
%           +R*mod(2*pi+mod(theta2+pi,2*pi)-mod(chie+pi/2,2*pi),2*pi);
        L = sqrt(ell^2-4*R^2); 
        L = L + R*mod(2*pi+mod(theta2,2*pi)-mod(chis-pi/2,2*pi),2*pi);
        L = L + R*mod(2*pi+mod(theta2+pi,2*pi)-mod(chie+pi/2,2*pi),2*pi);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% L = computeDubinsLSR(R, cls, cre, chis, chie)
%   Find the length of Left-Straight-Right Dubins car path.
%   Inputs:
%      R    - turn radius
%      cls  - center of left start circle
%      cre  - center of right end cirle
%      chis - start heading
%      chie - end heading
%    Ouputs:
%      L    - length of path
function L = computeDubinsLSR(R, cls, cre, chis, chie)
    ell = norm(cre(1:2)-cls(1:2));
    theta = atan2(cre(2)-cls(2),cre(1)-cls(1));
    theta2 = acos(2*R/ell);
    if isreal(theta2)==0,
        L = 99999999;
    else
        L = sqrt(ell^2-4*R^2); 
        L = L + R*mod(2*pi-mod(theta+theta2,2*pi) + mod(chis+pi/2,2*pi),2*pi);
        L = L + R*mod(2*pi-mod(theta+theta2-pi,2*pi)+mod(chie-pi/2,2*pi),2*pi);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% L = computeDubinsLSL(R, cls, cle, chis, chie)
%   Find the length of Left-Straight-Left Dubins car path.
%   Inputs:
%      R    - turn radius
%      cls  - center of left start circle
%      cle  - center of left end cirle
%      chis - start heading
%      chie - end heading
%    Ouputs:
%      L    - length of path
function L = computeDubinsLSL(R, cls, cle, chis, chie)
    theta = atan2(cle(2)-cls(2),cle(1)-cls(1));
    L = norm(cls(1:2)-cle(1:2)); 
    L = L + R*mod(2*pi-mod(theta+pi/2,2*pi)+mod(chis+pi/2,2*pi),2*pi);
    L = L + R*mod(2*pi-mod(chie+pi/2,2*pi)+mod(theta+pi/2,2*pi),2*pi);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% R = computeOptimalRadius(zs, chis, ze, chie, R_min, idx, k)
%   Find the radius such that max climb rate gets to end configuration
%   Inputs:
%      zs   - start position
%      chis - start heading
%      ze   - end position
%      chie - end heading
%      R_min - minimum turn radius
%      gam_max - maximum flight path angle
%      idx   - index of Dubins path
%      k     - number of spirals (must be at least 1)
%      hdist - distance that needs to be climbed
%    Ouputs:
%      R    - optimal turning radius
function R = computeOptimalRadius(zs, chis, ze, chie, R_min, gam_max, idx, k, hdist)

    R1 = R_min;
    %R2 = (k+1)/k*R_min;
    R2 = 2*R_min;
    R = (R1+R2)/2;
    switch(idx),
        case 1, %RSR
            error = 1;
            while abs(error) > .1, % geometric search to find R
                crs = zs + R*rotz(pi/2) *[cos(chis), sin(chis), 0]';
                cre = ze + R*rotz(pi/2) *[cos(chie), sin(chie), 0]';
                L = computeDubinsRSR(R, crs, cre, chis, chie);
                error = (L+2*pi*k*R) - abs(hdist)/tan(gam_max);
                if error > 0,
                    R2 = R;
                else
                    R1 = R;
                end
                R = (R1+R2)/2;
            end

        case 2, %RSL
            error = 1;
            while abs(error) > .1, % geometric search to find R
                crs = zs + R*rotz(pi/2) *[cos(chis), sin(chis), 0]';
                cle = ze + R*rotz(-pi/2)*[cos(chie), sin(chie), 0]';
                L = computeDubinsRSL(R, crs, cle, chis, chie);
                error = (L+2*pi*k*R)*tan(gam_max) - abs(hdist);
                if error > 0,
                    R2 = R;
                else
                    R1 = R;
                end
                R = (R1+R2)/2;
            end
            
        case 3, %LSR
            error = 1;
            while abs(error) > .1, % geometric search to find R
                cls = zs + R*rotz(-pi/2)*[cos(chis), sin(chis), 0]';
                cre = ze + R*rotz(pi/2) *[cos(chie), sin(chie), 0]';
                L = computeDubinsLSR(R, cls, cre, chis, chie);
                error = (L+2*pi*k*R)*tan(gam_max) - abs(hdist);
                if error > 0,
                    R2 = R;
                else
                    R1 = R;
                end
                R = (R1+R2)/2;
            end
            
        case 4, %LSL
            error = 1;
            while abs(error) > .1, % geometric search to find R
                cls = zs + R*rotz(-pi/2)*[cos(chis), sin(chis), 0]';
                cle = ze + R*rotz(-pi/2)*[cos(chie), sin(chie), 0]';
                L = computeDubinsLSL(R, cls, cle, chis, chie);
                error = (L+2*pi*k*R)*tan(gam_max) - abs(hdist);
                if error > 0,
                    R2 = R;
                else
                    R1 = R;
                end
                R = (R1+R2)/2;
            end      
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [zi, chii = addSpiralBeginning(zs, chis, ze, chie, R_min, gam_max, idx, hdist)
%   Add spiral at beginning of path to increase the path length
%   Inputs:
%      zs   - start position
%      chis - start heading
%      ze   - end position
%      chie - end heading
%      R_min - minimum turn radius
%      gam_max - maximum flight path angle
%      idx   - index of Dubins path
%      hdist - distance that needs to be climbed
%    Ouputs:
%      zi    - start position of intermediate spiral
%      chii  - start heading of intermediate spiral
function [zi,chii,L,ci,psii] = addSpiralBeginning(zs, chis, ze, chie, R_min, gam_max, idx, hdist)
    psi1 = 0;
    psi2 = 2*pi;
    psi   = (psi1+psi2)/2;
    switch(idx),
        case 1, %RLSR
            crs = zs + R_min*rotz(pi/2) *[cos(chis), sin(chis), 0]';
            cre = ze + R_min*rotz(pi/2) *[cos(chie), sin(chie), 0]';
            L = computeDubinsRSR(R_min, crs, cre, chis, chie);
            error = L - abs(hdist/tan(gam_max));
            while abs(error) > .1, % geometric search to find psi
                zi = crs + rotz(psi)*(zs-crs);
                chii = chis+psi;
                cli = zi + R_min*rotz(-pi/2) *[cos(chii), sin(chii), 0]';
                L = computeDubinsLSR(R_min, cli, cre, chii, chie);
                error = (L+abs(psi)*R_min) - abs(hdist/tan(gam_max));
                if error > 0,
                    psi2 = psi;
                else
                    psi1 = psi;
                end
                psi  = (psi1+psi2)/2;
            end
            zi   = crs + rotz(psi)*(zs-crs);
            chii = chis+psi;
            L = L + abs(psi)*R_min;
            ci = cli;
            psii = psi;


        case 2, %RLSL
            crs = zs + R_min*rotz(pi/2) *[cos(chis), sin(chis), 0]';
            cle = ze + R_min*rotz(-pi/2)*[cos(chie), sin(chie), 0]';
            L = computeDubinsRSL(R_min, crs, cle, chis, chie);
            error = L - abs(hdist/tan(gam_max));
            while abs(error) > .1, % geometric search to find psi
                zi = crs + rotz(psi)*(zs-crs);
                chii = chis+psi;
                cli = zi + R_min*rotz(-pi/2) *[cos(chii), sin(chii), 0]';
                L = computeDubinsLSL(R_min, cli, cle, chii, chie);
                error = (L+abs(psi)*R_min) - abs(hdist/tan(gam_max));
                if error > 0,
                    psi2 = psi;
                else
                    psi1 = psi;
                end
                psi  = (psi1+psi2)/2;
            end
            zi   = crs + rotz(psi)*(zs-crs);
            chii = chis+psi;
            L = L + abs(psi)*R_min;
            ci = cli;
            psii = psi;

        case 3, %LRSR
            cls = zs + R_min*rotz(-pi/2)*[cos(chis), sin(chis), 0]';
            cre = ze + R_min*rotz(pi/2) *[cos(chie), sin(chie), 0]';
            L = computeDubinsLSR(R_min, cls, cre, chis, chie);
            error = L - abs(hdist/tan(gam_max));
            while abs(error) > .1, % geometric search to find psi
                zi = cls + rotz(-psi)*(zs-cls);
                chii = chis-psi;
                cri = zi + R_min*rotz(pi/2) *[cos(chii), sin(chii), 0]';
                L = computeDubinsRSR(R_min, cri, cre, chii, chie);
                error = (L+abs(psi)*R_min) - abs(hdist/tan(gam_max));
                if error > 0,
                    psi2 = psi;
                else
                    psi1 = psi;
                end
                psi  = (psi1+psi2)/2;
            end
            zi   = cls + rotz(-psi)*(zs-cls);
            chii = chis-psi;
            L = L + abs(psi)*R_min;
            ci = cri;
            psii = psi;
             
        case 4, %LRSL
            cls = zs + R_min*rotz(-pi/2)*[cos(chis), sin(chis), 0]';
            cle = ze + R_min*rotz(-pi/2)*[cos(chie), sin(chie), 0]';
            L = computeDubinsLSL(R_min, cls, cle, chis, chie);
            error = L - abs(hdist/tan(gam_max));
            while abs(error) > .1, % geometric search to find psi
                zi = cls + rotz(-psi)*(zs-cls);
                chii = chis-psi;
                cri = zi + R_min*rotz(pi/2) *[cos(chii), sin(chii), 0]';
                L = computeDubinsRSL(R_min, cri, cle, chii, chie);
                error = (L+abs(psi)*R_min) - abs(hdist/tan(gam_max));
                if error > 0,
                    psi2 = psi;
                else
                    psi1 = psi;
                end
                psi  = (psi1+psi2)/2;
            end
            zi   = cls + rotz(-psi)*(zs-cls);
            chii = chis-psi;
            L = L + abs(psi)*R_min;
            ci = cri;
            psii = psi;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [zi, chii = addSpiralEnd(zs, chis, ze, chie, R_min, gam_max, idx, hdist)
%   Add spiral at end of path to increase the path length
%   Inputs:
%      zs   - start position
%      chis - start heading
%      ze   - end position
%      chie - end heading
%      R_min - minimum turn radius
%      gam_max - maximum flight path angle
%      idx   - index of Dubins path
%      hdist - distance that needs to be climbed
%    Ouputs:
%      zi    - start position of intermediate spiral
%      chii  - start heading of intermediate spiral
function [zi,chii,L,ci,psii] = addSpiralEnd(zs, chis, ze, chie, R_min, gam_max, idx, hdist)
    psi1 = 0;
    psi2 = 2*pi;
    psi   = (psi1+psi2)/2;
    
    switch(idx),
        case 1, %RSLR
            crs = zs + R_min*rotz(pi/2) *[cos(chis), sin(chis), 0]';
            cre = ze + R_min*rotz(pi/2) *[cos(chie), sin(chie), 0]';
            L = computeDubinsRSR(R_min, crs, cre, chis, chie);
            error = L - abs(hdist/tan(gam_max));
            while abs(error) > .1, % geometric search to find psi
                zi = cre + rotz(-psi)*(ze-cre);
                chii = chie-psi;
                cli = zi + R_min*rotz(-pi/2) *[cos(chii), sin(chii), 0]';
                L = computeDubinsRSL(R_min, crs, cli, chis, chii);
                error = (L+abs(psi)*R_min) - abs(hdist/tan(gam_max));
                if error > 0,
                    psi2 = psi;
                else
                    psi1 = psi;
                end
                psi  = (psi1+psi2)/2;
            end
            zi   = cre + rotz(-psi)*(ze-cre);
            chii = chie-psi;
            L = L + abs(psi)*R_min;
            ci = cli;
            psii = psi;


        case 2, %RSRL
            crs = zs + R_min*rotz(pi/2) *[cos(chis), sin(chis), 0]';
            cle = ze + R_min*rotz(-pi/2)*[cos(chie), sin(chie), 0]';
            L = computeDubinsRSL(R_min, crs, cle, chis, chie);
            error = L - abs(hdist/tan(gam_max));
            while abs(error) > .1, % geometric search to find psi
                zi = cle + rotz(psi)*(ze-cle);
                chii = chie+psi;
                cri = zi + R_min*rotz(pi/2)*[cos(chii), sin(chii), 0]';
                L = computeDubinsRSR(R_min, crs, cri, chis, chii);
                error = (L+abs(psi)*R_min) - abs(hdist/tan(gam_max));
                if error > 0,
                    psi2 = psi;
                else
                    psi1 = psi;
                end
                psi  = (psi1+psi2)/2;
            end
            zi   = cle + rotz(psi)*(ze-cle);
            chii = chie+psi;
            cri  = zi + R_min*rotz(pi/2) *[cos(chii), sin(chii), 0]';
            L    = L + abs(psi)*R_min;
            ci   = cri;
            psii = psi;

        case 3, %LSLR
            cls = zs + R_min*rotz(-pi/2)*[cos(chis), sin(chis), 0]';
            cre = ze + R_min*rotz(pi/2) *[cos(chie), sin(chie), 0]';
            L = computeDubinsLSR(R_min, cls, cre, chis, chie);
            error = L - abs(hdist/tan(gam_max));
            while abs(error) > .1, % geometric search to find psi
                zi = cre + rotz(-psi)*(ze-cre);
                chii = chie-psi;
                cli = zi + R_min*rotz(-pi/2) *[cos(chii), sin(chii), 0]';
                L = computeDubinsLSL(R_min, cls, cli, chis, chii);
                error = (L+abs(psi)*R_min) - abs(hdist/tan(gam_max));
                if error > 0,
                    psi2 = psi;
                else
                    psi1 = psi;
                end
                psi  = (psi1+psi2)/2;
            end
            zi   = cre + rotz(-psi)*(ze-cre);
            chii = chie-psi;
            L = L + abs(psi)*R_min;
            ci = cli;
            psii = psi;
             
        case 4, %LSRL
            cls = zs + R_min*rotz(-pi/2)*[cos(chis), sin(chis), 0]';
            cle = ze + R_min*rotz(-pi/2)*[cos(chie), sin(chie), 0]';
            L = computeDubinsLSL(R_min, cls, cle, chis, chie);
            error = L - abs(hdist/tan(gam_max));
            while abs(error) > .1, % geometric search to find psi
                zi = cle + rotz(psi)*(ze-cle);
                chii = chie+psi;
                cri = zi + R_min*rotz(pi/2) *[cos(chii), sin(chii), 0]';
                L = computeDubinsLSR(R_min, cls, cri, chis, chii);
                error = (L+abs(psi)*R_min) - abs(hdist/tan(gam_max));
                if error > 0,
                    psi2 = psi;
                else
                    psi1 = psi;
                end
                psi  = (psi1+psi2)/2;
            end
            zi   = cle + rotz(psi)*(ze-cle);
            chii = chie+psi;
            L = L + abs(psi)*R_min;
            ci = cri;
            psii = psi;
    end
end
