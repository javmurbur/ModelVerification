function [x,y,z] = rotateGFtoBF (X,Y,Z,roll,pitch,yaw)
% Necesaria para pasar vectores en ejes globales a ejes cuerpo 
% Rotación con respecto a ejes fijos

cr = cos(roll);
sr = sin(roll);
cp = cos(pitch);
sp = sin(pitch);
cy = cos(yaw);
sy = sin(yaw);

% Rotación en el eje xb(roll)
Rx = [1 0 0;0 cr -sr;0 sr cr];

% Rotación en el eje yb(pitch)
Ry = [cp 0 sp;0 1 0;-sp 0 cp];

%Rotación en el eje zb(yaw)
Rz = [cy -sy 0;+sy cy 0;0 0 1];

RGB = Rz*Ry*Rx;

 x = RGB(1,:)*[X;Y;Z];
 y = RGB(2,:)*[X;Y;Z];
 z = RGB(3,:)*[X;Y;Z];
end