clear all
close all

gamma=30;   % [deg]

alpha11 = -175; %[deg]
alpha12 = 175;  %[deg]
Nalpha1 = 10;
alpha21 = -170;  %[deg]
alpha22 = 170;   %[deg]
Nalpha2 = 7;

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

