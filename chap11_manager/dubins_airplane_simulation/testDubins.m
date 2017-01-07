
switch(9),
    case 1, % short climb RSR
        start_node = [0,   0,   -100,   0*pi/180,    P.Va0];
        end_node   = [0, 200,   -125, 270*pi/180,    P.Va0];
    case 2, % short climb RSL
        start_node = [0,   0,    -100, -70*pi/180,    P.Va0];
        end_node   = [100, 100,  -125, -70*pi/180,    P.Va0];
    case 3, % short climb LSR
        start_node = [0,   0,    -100, 70*pi/180,    P.Va0];
        end_node   = [100, -100, -125, 70*pi/180,    P.Va0];
    case 4, % short climb LSL
        start_node = [0,   0,    -100,  70*pi/180,   P.Va0];
        end_node   = [100, -100, -125, -135*pi/180,  P.Va0];
    case 5, % long climb RSR
        start_node = [0,   0,   -100,   0*pi/180,    P.Va0];
        end_node   = [0, 200,   -250, 270*pi/180,    P.Va0];
    case 6, % long climb RSL
        start_node = [0,   0,    -100, -70*pi/180,    P.Va0];
        end_node   = [100, 100,  -350, -70*pi/180,    P.Va0];
    case 7, % long climb LSR
        start_node = [0,   0,    -350, 70*pi/180,    P.Va0];
        end_node   = [100, -100, -100, 70*pi/180,    P.Va0];
    case 8, % long climb LSL
        start_node = [0,   0,    -350,  70*pi/180,   P.Va0];
        end_node   = [100, -100, -100, -135*pi/180,  P.Va0];
    case 9, % intermediate climb RLSR (climb at beginning)
        start_node = [0,   0,   -100,   0*pi/180,     P.Va0];
        end_node   = [0, 200,   -200, 270*pi/180,    P.Va0];
    case 10, % intermediate climb RLSL (climb at beginning)
        start_node = [0,   0,   -100, 0*pi/180,     P.Va0];
        end_node   = [100, 100, -200, -90*pi/180,   P.Va0];
    case 11, % intermediate climb LRSR (climb at beginning)
        start_node = [0,   0,   -100, 0*pi/180,     P.Va0];
        end_node   = [100, -100, -200, 90*pi/180,    P.Va0];
    case 12, % intermediate climb LRSL (climb at beginning)
        start_node = [0,   0,   -100, 0*pi/180,     P.Va0];
        end_node   = [100, -100, -200, -90*pi/180,   P.Va0];
    case 13, % intermediate climb RSLR (descend at end)
        start_node = [0,   0,   -200, 0*pi/180,     P.Va0];
        end_node   = [100, 100, -100, 90*pi/180,    P.Va0];
    case 14, % intermediate climb RSRL (descend at end) 
        start_node = [0,   0,   -200, 0*pi/180,     P.Va0];
        end_node   = [100, 100, -100, -90*pi/180,   P.Va0];
    case 15, % intermediate climb LSLR (descend at end)
        start_node = [0,   0,   -200, 70*pi/180,     P.Va0];
        end_node   = [100, -100, -100, 90*pi/180,    P.Va0];
    case 16, % intermediate climb LSRL (descend at end)
        start_node = [0,   0,   -150, 0*pi/180,     P.Va0];
        end_node   = [100, -100,-100, -90*pi/180,   P.Va0];
end

R_min = 40;
gam_max = 10*pi/180;

dubinspath = dubinsParameters(start_node, end_node, R_min, gam_max);

r_dubins=drawDubinsAirplanePath(dubinspath);
