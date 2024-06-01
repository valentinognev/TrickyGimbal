clear all
close all

gamma=30;   % [deg]

psi1=20;  % [deg]
beta1=15; % [deg]
psi2=-10;  % [deg]
beta2=-15; % [deg]

rotpsi1=rotx(psi1/180*pi);
rotbeta1=roty(beta1/180*pi);
rotpsi2=rotx(psi2/180*pi);
rotbeta2=roty(beta2/180*pi);

rot1=rotbeta1*rotpsi1;
rot2=rotbeta2*rotpsi2;

q1=rotm2quat(rot1);
q2=rotm2quat(rot2);

q1_=eul2quat([psi1/180*pi,beta1/180*pi,0],'XYZ');
q2_=eul2quat([psi2/180*pi,beta2/180*pi,0],'XYZ');

alpha1=linspace(alpha11, alpha12, Nalpha1);
alpha2=linspace(alpha21, alpha22, Nalpha2);
[Alpha1, Alpha2]=meshgrid(alpha1,alpha2);

fffd2=asin(sind(Alpha2/2)*sind(gamma))/pi*180;  % angle with Z axis

eta=acos((sind(Alpha2/2)-sind(gamma)*sind(fffd2))./(cosd(fffd2)*cosd(gamma)))/pi*180;
phi=(180-Alpha1-eta);
fff=2*fffd2;
R=1;
X=R*sind(fff).*cosd(180-phi);
Y=R*sind(fff).*sind(180-phi);
Z=R*cosd(fff);
surf(X,Y,Z)

