%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\
%??????
syms theta phi psi
%    phi   = x(7);      % roll angle (degrees)   
%    theta = x(8);      % pitch angle (degrees)
%    psi   = x(9);      % yaw angle (degrees)
% picth --- theta
% roll  --- phi
% yaw   --- psi
%% body to Nav
C11=cos(theta)*cos(psi);
C12=sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
C13=cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
C21=cos(theta)*sin(psi);
C22=sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
C23=cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
C31=-sin(theta);
C32=sin(phi)*cos(theta);
C33=cos(phi)*cos(theta);

Cnb=[C11,C12,C13;C21,C22,C23;C31,C32,C33]
%% ???-????
T11=cos(theta)*cos(psi);        %T11=C11
T12=cos(theta)*sin(psi);        %T12=C21
T13=-sin(theta);                %T13=C31
T21=sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);   %T21=C12
T22=sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);   %T22=C22
T23=sin(phi)*cos(theta);        %T23=C32
T31=cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);   %T31=C13
T32=cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);   %T32=C23
T33=cos(phi)*cos(theta);        %T33=C33

Cbn=[T11,T12,T13;T21,T22,T23;T31,T32,T33]          %Cbn=Cnb'
%% ????????????????
syms w_ns w_es w_ds 
P = Cbn*[w_ns w_es w_ds]'

