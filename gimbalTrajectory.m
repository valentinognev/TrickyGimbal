function gimbalTrajectory
% close all

gamma=20/180*pi;   % [deg]

elevation1=0.67;%deg2rad(-20);  % [rad] elevation angle with Z - phi
azimuth1=-1.23;%deg2rad(40);  % [rad] azimuth angle in XY plane - curlphi
elevation2=0.69; %deg2rad(30); % [rad] elevation angle with Z - phi
azimuth2=1.96;%deg2rad(70);  % [rad] azimuth angle in XY plane - curlphi

rotElev1=roty(rad2deg(elevation1));   % input to totx, roty, rotz is in degrees 
rotAz1=rotz(rad2deg(azimuth1));

rotElev2=roty(rad2deg(elevation2));
rotAz2=rotz(rad2deg(azimuth2));

rot1=rotAz1*rotElev1;
rot2=rotAz2*rotElev2;

rinit=[0;0;1];
r1=rot1*rinit;
r2=rot2*rinit;

r1r2axis=cross(r1,r2); r1r2axis=r1r2axis/norm(r1r2axis);
theta=acos(dot(r1,r2));
N=10;
dtheta=theta/N;

rotdtheta=axang2rotm([r1r2axis',dtheta]);

% figure(1)
% plot3([0,r1(1)],[0,r1(2)],[0,r1(3)],'rx-'); hold on; grid on; axis equal
% plot3([0,r2(1)],[0,r2(2)],[0,r2(3)],'mx-'); 
% xlabel('x');
% ylabel('y');
% zlabel('z');

curr=r1;
for i=1:N
    elevation(i)=acos(dot(curr,[0,0,1]));
    azimuth(i)=atan2(curr(2),curr(1));
    alpha2(i)=2*asin(sin(elevation(i)/2)/sin(gamma));
    eta=acos((sin(alpha2(i)/2)-sin(gamma)*sin(elevation(i)/2))/(cos(elevation(i)/2)*cos(gamma)));
    alpha1(i)=pi-eta-azimuth(i);

    curr_2=[(-cos(alpha1(i)))*cos(gamma)*sin(gamma) - ((-cos(alpha1(i)))*cos(alpha2(i))*cos(gamma) - sin(alpha1(i))*sin(alpha2(i)))*sin(gamma), ...
        cos(gamma)*sin(alpha1(i))*sin(gamma) - (cos(alpha2(i))*cos(gamma)*sin(alpha1(i)) - cos(alpha1(i))*sin(alpha2(i)))*sin(gamma),...
        cos(gamma)^2 + cos(alpha2(i))*sin(gamma)^2];

    figure(1)
    plot3([0,curr(1)],[0,curr(2)],[0,curr(3)],'b-o');
    plotGimbal(alpha1(i), alpha2(i), gamma, 1);
 
    disp (sprintf('elevation %f , azimuth %f , alpha1 %f , alpha2 %f',elevation(i),azimuth(i),alpha1(i),alpha2(i)) );
    curr=rotdtheta*curr;
    disp ''
end
figure(2);
plot(elevation,'x-'); hold on
plot(azimuth,'x-'); hold on

%%
function plotGimbal(alpha1, alpha2, gamma, fignum)
if nargin<3
    fignum = 1;
end
rot1Gamma=roty(rad2deg(gamma));
rrod1=[-1,0,0];
rotAlpha1 = @(alpha1) rotz(rad2deg(alpha1));   % input in radians
rrod2=[1,0,0];
rotAlpha2 = @(alpha2) rotz(rad2deg(alpha2));   % input in radians
rrodCam=[-1,0,0];
rotCam = roty(rad2deg(pi/2-gamma));

figure(fignum); hold on; grid on;
plotCoordSystem([0,0,0], eye(3), .1);

rod1rot=rotAlpha1(pi-alpha1)*rot1Gamma;
rod1tip=rod1rot*rrod1';
rod1=[[0,0,0];rod1tip'];
plot3(rod1(:,1), rod1(:,2), rod1(:,3));
plotCoordSystem(rod1tip', rod1rot, .1);

rod2rot=rod1rot*rotAlpha2(alpha2);
rod2tip=rod2rot*rrod2';
rod2=[[0,0,0];rod2tip']+[1;1]*rod1(2,:);
plot3(rod2(:,1), rod2(:,2), rod2(:,3));
plotCoordSystem(rod2(2,:), rod2rot, .1);

rodCamrot=rod2rot*rotCam;
rodCamtip=rodCamrot*rrodCam';
rodCam=[[0,0,0];rodCamtip']+[1;1]*rod2(2,:);
plot3(rodCam(:,1), rodCam(:,2), rodCam(:,3));
plotCoordSystem(rodCam(2,:), rodCamrot, .1);
axis('equal')
% plot3(rodCamtip(1),rodCamtip(2),rodCamtip(3),'rx');

fff=acos(dot(rodCamtip,[0,0,1])/norm(rodCamtip)/norm([0,0,1]));
phi_=atan2(rodCamtip(2),rodCamtip(1))
disp ''
%%
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
%%