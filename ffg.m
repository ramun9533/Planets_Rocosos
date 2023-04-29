close all
clear
clc

% Definir coordenadas del centro de la esfera (el sol) y su radio
center = [0 0 0]; % coordenadas del centro
radius = 5; % radio

t = linspace(0,2*pi,100);
xti = cos(2*t);
yti = sin(2*t);
xme = cos(8.2*t); %traslacion trayectoria
yme = sin(8.2*t);%traslacion trayectoria
xv = cos(3.2*t);
yv = sin(3.2*t);
xma = cos(1.06*t);
yma = sin(1.06*t);
z = zeros(1,100);

[xp0, yp0, zp0] = sphere;% Crear matriz de coordenadas de la esfera (el sol)
xp0 = (radius*.5) * xp0 + center(1);
yp0 = (radius*.5) * yp0 + center(2);
zp0 = (radius*.5) * zp0 + center(3);

[xp, yp, zp] = sphere;%Mercurio D = 4879 Km, 88 dias orbita / 4
xp = (radius*0.38) * xp + center(1);
yp = (radius*0.38) * yp + center(2);
zp = (radius*0.38) * zp + center(3);
[xp2, yp2, zp2] = sphere;%Venus D = 12,104 Km, 225 dias / 1.6
xp2 = (radius*0.94) * xp2 + center(1);
yp2 = (radius*0.94) * yp2 + center(2);
zp2 = (radius*0.94) * zp2 + center(3);
[xp3, yp3, zp3] = sphere;%Tierra D = 12, 742 Km, 365 * 1
xp3 = (radius*1) * xp3 + center(1);
yp3 = (radius*1) * yp3 + center(2);
zp3 = (radius*1) * zp3 + center(3);
[xp4, yp4, zp4] = sphere;%Marte 6,779 Km, 687, * .53
xp4 = (radius*0.53) * xp4 + center(1);
yp4 = (radius*0.53) * yp4 + center(2);
zp4 = (radius*0.53) * zp4 + center(3);

psi = pi/12;
theta = pi/6;
phi = pi/9;
R = 3 * [cos(phi)*cos(theta) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);    sin(phi)*cos(theta) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);    -sin(theta) cos(theta)*sin(psi) cos(theta)*cos(psi)];
p = [4.6*xme;6.98*yme;z];
p = R*p;

%z2 = zeros(1,150);
p2 = [10.89*xv;10.75*yv;z];%Esta variable modifica la figura de e inclinacion de la trayectoria esfera de la orbita
p2 = R*p2;
%z3 = zeros(1,200);
p3 = [15.21*xti;14.71.*yti;z];%Esta variable modifica la figura de e inclinacion de la trayectoria esfera de la orbita
p3 = R*p3;
%z4 = zeros(1,220);
p4 = [22.49*xma;20.67*yma;z];%Esta variable modifica la figura de e inclinacion de la trayectoria esfera de la orbita
p4 = R*p4;
% a=VideoWriter('sistemasolar_3d','MPEG-4');
%open(a);
for idx=1:length(t);
    figure(1)
    clf
    %Mercurio (floor promedia hacia abajo)
    %Venus
    %Marte

    plot3(p(1,:),p(2,:),p(3,:),'r')
    hold on
    surf(xp+p(1,idx),yp+p(2,idx),zp+p(3,idx))%Mercurio
    plot3(p2(1,:),p2(2,:),p2(3,:),'k')
    surf(xp2+p2(1,idx),yp2+p2(2,idx),zp2+p2(3,idx))%Venus
    plot3(p3(1,:),p3(2,:),p3(3,:),'g')
    surf(xp3+p3(1,idx),yp3+p3(2,idx),zp3+p3(3,idx))%Tierra
    plot3(p4(1,:),p4(2,:),p4(3,:),'b')
    surf(xp4+p4(1,idx),yp4+p4(2,idx),zp4+p4(3,idx))%Marte
    % Graficar la esfera central (el sol)
    surf(xp0, yp0, zp0, 'FaceColor', [1 1 0], 'EdgeColor', 'none')
    xlim([-70 70])
    ylim([-70 70])
    zlim([-70 70])
    view(100,100)
    pause(0.1)
    %writeVideo(a,getframe(gcf));
end
%close (a)
