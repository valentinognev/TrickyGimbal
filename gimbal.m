function gimbal
close all

drawArrow3 = @(x,y,z,varargin) quiver3( x(1),y(1),z(1), x(2)-x(1),y(2)-y(1),z(2)-z(1), varargin{:} ); 
drawArrow3o = @(vec,varargin) quiver3( 0,0,0,vec(1),vec(2),vec(3), varargin{:}); 

alpha1=deg2rad(44);  %deg
alpha2=deg2rad(90); %deg
gamma=deg2rad(20);   %deg

vecz=[0,0,1]';

rotg1=roty(rad2deg(gamma));
g1vecz=rotg1*vecz;


rota1=rotz(rad2deg(alpha1));
a1g1vecz=rota1*g1vecz;

CA=a1g1vecz;
beta=pi/2-alpha2/2;

% figure(1)
% drawArrow3o(vecz); hold on; grid on; axis equal; xlabel('x'); ylabel('y');zlabel('z');
% drawArrow3o(g1vecz);
% drawArrow3o(CA);

rotb = axang2rotm([CA', -alpha2]);

vecmx=[-1,0,0]';
g1vecmx=rotg1*vecmx;
a1g1vecmx=rota1*g1vecmx;
CO=a1g1vecmx;
O=[0,0,0];
C=O+CO';
CO_=[O;C];
plot3(CO_(:,1),CO_(:,2),CO_(:,3));hold on;grid on;

CB=rotb*-CO;
B=CO+CB;
CB_=[C;B'];
plot3(CB_(:,1), CB_(:,2), CB_(:,3));hold on;grid on;

BO=B;
EO=[BO(1),BO(2),0];
phi=rad2deg(atan(BO(2)/BO(1)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rot1Gamma=roty(rad2deg(gamma));
rrod1=[-1,0,0];
rotAlpha1 = @(alpha1) rotz(rad2deg(alpha1));   % input in radians
rrod2=[1,0,0];
rotAlpha2 = @(alpha2) rotz(rad2deg(alpha2));   % input in radians
rrodCam=[-1,0,0];
rotCam = roty(rad2deg(pi/2-gamma));

i=1;
figure(1); hold on; grid on;
plotCoordSystem([0,0,0], eye(3), .1);

rod1rot=rotAlpha1(alpha1(i))*rot1Gamma;
rod1tip=rod1rot*rrod1';
rod1=[[0,0,0];rod1tip'];
plot3(rod1(:,1), rod1(:,2), rod1(:,3));
plotCoordSystem(rod1tip', rod1rot, .1);

rod2rot=rod1rot*rotAlpha2(-alpha2(i));
rod2tip=rod2rot*rrod2';
rod2=[[0,0,0];rod2tip']+[1;1]*rod1(2,:);
plot3(rod2(:,1), rod2(:,2), rod2(:,3));
plotCoordSystem(rod2(2,:), rod2rot, .1);

rodCamrot=rod2rot*rotCam;
rodCamtip=rodCamrot*rrodCam';
rodCam=[[0,0,0];rodCamtip']+[1;1]*rod2(2,:);
plot3(rodCam(:,1), rodCam(:,2), rodCam(:,3));
plotCoordSystem(rodCam(2,:), rodCamrot, .1);

plot3(rodCamtip(1),rodCamtip(2),rodCamtip(3),'rx')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fffd2=asin(sin(alpha2/2)*sin(gamma));
d=1;
CO = d;
BC = CO;
BO = 2*CO*sin(alpha2/2);
theta=fffd2;
BE=BO*sin(theta);
EO=BO*cos(theta);
FO=CO*cos(gamma);
FC=CO*sin(gamma);
DE=FC;
BD=BE-DE;
DC=sqrt(BC^2-BD^2);
EF=DC;
eta=acos((FO^2+EO^2-EF^2)/2/FO/EO);


eta2=acos((sin(alpha2/2)-sin(gamma)*sin(fffd2))/(cos(fffd2)*cos(gamma)));

% eta=acos(sqrt((sind(alpha2/2)^2-sind(gamma)^2)/(cosd(gamma)^2)))/pi*180;
phi2=rad2deg(pi-alpha1-eta);

% figure(2)
% drawArrow3o(vecmx); hold on; grid on; axis equal; xlabel('x'); ylabel('y');zlabel('z');
% drawArrow3o(g1vecmx);
% drawArrow3o(CO);
% drawArrow3o(CA);
% drawArrow3o(BO);
% drawArrow3o(EO);
disp (['phi vec ',num2str(phi),'   phi2 ', num2str(phi2)]);


function plotCoordSystem(pos,rotMat,scale)
if nargin < 3
    scale=1;
    if nargin < 2
        rotMat=eye(3);
    end
end
points=[0, 0, 0;
        1, 0, 0;
        0, 1, 0;
        0, 0, 1]'*scale;
points=(rotMat*points+(ones(4,1)*pos)')';
origin=points(1,:);
dirX=points(2,:)-points(1,:);
dirY=points(3,:)-points(1,:);
dirZ=points(4,:)-points(1,:);
quiver3(origin(1),origin(2),origin(3),dirX(1),dirX(2),dirX(3),'LineWidth',2,'Color','b');hold on;axis equal;
quiver3(origin(1),origin(2),origin(3),dirY(1),dirY(2),dirY(3),'LineWidth',2,'Color','r');
quiver3(origin(1),origin(2),origin(3),dirZ(1),dirZ(2),dirZ(3),'LineWidth',2,'Color','m');
xlabel('X');ylabel('Y');zlabel('Z');


