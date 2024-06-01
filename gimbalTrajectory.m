clear all
close all

gamma=30;   % [deg]

elevation1=20;  % [deg] elevation angle with Z - phi
azimuth1=0;  % [deg] azimuth angle in XY plane - curlphi
elevation2=30;  % [deg] elevation angle with Z - phi
azimuth2=70;  % [deg] azimuth angle in XY plane - curlphi

rotElev1=roty(elevation1);   % input to totx, roty, rotz is in degrees 
rotAz1=rotz(azimuth1);

rotElev2=roty(elevation2);
rotAz2=rotz(azimuth2);

rot1=rotAz1*rotElev1;
rot2=rotAz2*rotElev2;

% q1=rotm2quat(rot1);
% q2=rotm2quat(rot2);

% q1_=eul2quat([0,-elevation1,-azimuth1]/180*pi,'ZYX');
% q2_=eul2quat([azimuth2,elevation2,0]/180*pi,'XYZ');
rinit=[0;0;1];
r1=rot1*rinit;
r2=rot2*rinit;

r1r2axis=cross(r1,r2); r1r2axis=r1r2axis/norm(r1r2axis);
theta=acos(dot(r1,r2));
N=10;
dtheta=theta/N;

rotdtheta=axang2rotm([r1r2axis',dtheta]);

figure(1)
plot3([0,r1(1)],[0,r1(2)],[0,r1(3)],'rx-'); hold on; grid on; axis equal
plot3([0,r2(1)],[0,r2(2)],[0,r2(3)],'mx-'); 
xlabel('x');
ylabel('y');
zlabel('z');

curr=r1;
for i=1:N
    curr=rotdtheta*curr;
    plot3([curr(1)],[curr(2)],[curr(3)],'bo'); 
    elevation(i)=acos(dot(curr,[0,0,1]));
    azimuth(i)=atan2(curr(2),curr(1));
    alpha2(i)=2*asin(sin(elevation(i)/2)/sin(gamma));
    eta=acos((sind(alpha2(i)/2)-sind(gamma)*sin(elevation(i)/2))/(cos(elevation(i)/2)*cosd(gamma)))/pi*180;
    alpha1(i)=pi-eta-azimuth(i);
end
figure(2);
plot(elevation)
