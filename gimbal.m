clear all
close all

drawArrow3 = @(x,y,z,varargin) quiver3( x(1),y(1),z(1), x(2)-x(1),y(2)-y(1),z(2)-z(1), varargin{:} ); 
drawArrow3o = @(vec,varargin) quiver3( 0,0,0,vec(1),vec(2),vec(3), varargin{:}); 

alpha1=45;  %deg
alpha2=34; %deg
gamma=15;   %deg

vecz=[0,0,1]';

rotg1=roty(gamma);
g1vecz=rotg1*vecz;


rota1=rotz(alpha1);
a1g1vecz=rota1*g1vecz;

CA=a1g1vecz;
beta=pi/2-(alpha2/180*pi)/2;

% figure(1)
% drawArrow3o(vecz); hold on; grid on; axis equal; xlabel('x'); ylabel('y');zlabel('z');
% drawArrow3o(g1vecz);
% drawArrow3o(CA);

rotb = axang2rotm([CA', beta]);

vecmx=[-1,0,0]';
g1vecmx=rotg1*vecmx;
a1g1vecmx=rota1*g1vecmx;
CO=a1g1vecmx;
BO=rotb*CO;
EO=[BO(1),BO(2),0];
phi=atan(BO(2)/BO(1))/pi*180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fffd2=asin(sind(alpha2/2)*sind(gamma));
d=1;
CO = d;
BC = CO;
BO = 2*CO*sind(alpha2/2);
theta=fffd2;
BE=BO*sin(theta);
EO=BO*cos(theta);
FO=CO*cosd(gamma);
FC=CO*sind(gamma);
DE=FC;
BD=BE-DE;
DC=sqrt(BC^2-BD^2);
EF=DC;
eta=acos((FO^2+EO^2-EF^2)/2/FO/EO)/pi*180;


eta2=acos((sind(alpha2/2)-sind(gamma)*sin(fffd2))/(cos(fffd2)*cosd(gamma)))/pi*180;

% eta=acos(sqrt((sind(alpha2/2)^2-sind(gamma)^2)/(cosd(gamma)^2)))/pi*180;
phi2=(180-alpha1-eta);

% figure(2)
% drawArrow3o(vecmx); hold on; grid on; axis equal; xlabel('x'); ylabel('y');zlabel('z');
% drawArrow3o(g1vecmx);
% drawArrow3o(CO);
% drawArrow3o(CA);
% drawArrow3o(BO);
% drawArrow3o(EO);
disp (['phi vec ',num2str(phi),'   phi2 ', num2str(phi2)]);


