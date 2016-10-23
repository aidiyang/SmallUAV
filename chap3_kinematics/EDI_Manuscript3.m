%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\
%定义旋转矩阵
syms theta phi psi
%    phi   = x(7);      % roll angle (degrees)   
%    theta = x(8);      % pitch angle (degrees)
%    psi   = x(9);      % yaw angle (degrees)
%θ：俯仰角 picth  theta
%φ或者r：横滚角 roll  phi
%ψ：航向角yaw  psi
C11=cos(theta)*cos(psi);
C12=sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
C13=cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);

C21=cos(theta)*sin(psi);
C22=sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
C23=cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);

C31=-sin(theta);
C32=sin(phi)*cos(theta);
C33=cos(phi)*cos(theta);

%机体系-》导航系
Cnb=[C11,C12,C13;C21,C22,C23;C31,C32,C33]
%% 求机体下的速度到导航系下变换
syms u v w 
P = Cnb*[u v w]'



conj(w)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - conj(v)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + cos(psi)*cos(theta)*conj(u)
 conj(v)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - conj(w)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + cos(theta)*conj(u)*sin(psi)
                                                                        cos(phi)*cos(theta)*conj(w) - conj(u)*sin(theta) + cos(theta)*conj(v)*sin(phi)

