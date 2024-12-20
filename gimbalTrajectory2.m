function gimbalTrajectory2
% close all

gamma=20/180*pi;   % [deg]

% elevation - from -30 to 30
% azimuth - from -30 to 30

elevation1=deg2rad(-20);  %0.67;% [rad] elevation angle with Z - phi
azimuth1=deg2rad(40);  %-1.23;% [rad] azimuth angle in XY plane - curlphi
elevation2=deg2rad(30); %0.69; % [rad] elevation angle with Z - phi
azimuth2=deg2rad(70);  %1.96;% [rad] azimuth angle in XY plane - curlphi

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

curr=r1;
for i=1:N
    elevation_1(i)=acos(dot(curr,[0,0,1]));
    azimuth_1(i)=atan2(curr(2),curr(1));
    alpha2_1(i)=2*asin(sin(elevation_1(i)/2)/sin(gamma));
    eta=acos((sin(alpha2_1(i)/2)-sin(gamma)*sin(elevation_1(i)/2))/(cos(elevation_1(i)/2)*cos(gamma)));
    alpha1_1(i)=pi-eta-azimuth_1(i);
    plot3([0,curr(1)],[0,curr(2)],[0,curr(3)],'b-o');
    plotGimbal(alpha1_1(i), alpha2_1(i), gamma, 1);

    elevation(i)=pi/2-acos(dot(curr,[0,1,0]));
    azimuth(i)=atan2(curr(3),curr(1));

    curr_2=[(-cos(alpha1(i)))*cos(gamma)*sin(gamma) - ((-cos(alpha1(i)))*cos(alpha2(i))*cos(gamma) - sin(alpha1(i))*sin(alpha2(i)))*sin(gamma), ...
        cos(gamma)*sin(alpha1(i))*sin(gamma) - (cos(alpha2(i))*cos(gamma)*sin(alpha1(i)) - cos(alpha1(i))*sin(alpha2(i)))*sin(gamma),...
        cos(gamma)^2 + cos(alpha2(i))*sin(gamma)^2];

    p_1=[sin(elevation_1(i))*cos(azimuth_1(i)), ...
        sin(elevation_1(i))*sin(azimuth_1(i)), ...
        cos(elevation_1(i))];
    p=  [cos(elevation(i))*cos(azimuth(i)), ...
         sin(elevation(i)), ...
         cos(elevation(i))*sin(azimuth(i))];

    elevation(i);
    azimuth(i);

    test_s = @(elev_s, az_s) [cos(az_s),                 ...                           
                              sin(az_s),                 ...         
                              tan(az_s),                 ...                                  
                              sin(elevation_1(i))*cos(azimuth_1(i)), ...
                              sin(elevation_1(i))*sin(azimuth_1(i)), ...
                              cos(elevation_1(i))]...
                              -...
                             [sin(elevation_1(i))*cos(azimuth_1(i))/cos(elev_s), ...
                             cos(elevation_1(i))/cos(elev_s), ...
                             cos(elevation_1(i))/(sin(elevation_1(i))*cos(azimuth_1(i))), ...
                             cos(elev_s)*cos(az_s),              ...   
                             sin(elev_s),                        ...   
                             cos(elev_s)*sin(az_s)];

    elev_s(i)=asin(sin(elevation_1(i))*sin(azimuth_1(i)));
    az_s(i)=acos(sin(elevation_1(i))*cos(azimuth_1(i))/cos(elev_s(i)));
    az_s_2(i)=asin(cos(elevation_1(i))/cos(elev_s(i)));
    az_s_3(i)=atan(cos(elevation_1(i))/(sin(elevation_1(i))*cos(azimuth_1(i))));

    angs_s = [rad2deg(elevation(i)),rad2deg(azimuth(i)), test_s(elevation(i),azimuth(i));...
              rad2deg(elev_s(i)),rad2deg(az_s(i)), test_s(elev_s(i),az_s(i));...
              rad2deg(elev_s(i)),rad2deg(az_s_2(i)),test_s(elev_s(i),az_s_2(i));...
              rad2deg(elev_s(i)),rad2deg(az_s_3(i)), test_s(elev_s(i),az_s_3(i))];

    test_1 = @(elev_1, az_1) [cos(az_1),                         ...               
                              sin(az_1),                      ...  
                              tan(az_1),                   ...                          
                              sin(elev_1)*cos(az_1),    ...         
                              sin(elev_1)*sin(az_1), ...
                              cos(elev_1)]...
                              -...
                             [cos(elevation(i))*cos(azimuth(i))/sin(elev_1), ...
                              sin(elevation(i))/sin(elev_1), ...
                              sin(elevation(i))/(cos(elevation(i))*cos(azimuth(i))), ...
                              cos(elevation(i))*cos(azimuth(i)), ...
                              sin(elevation(i)),     ...
                              cos(elevation(i))*sin(azimuth(i))];

    elevation_1(i);
    azimuth_1(i);
    elev_1(i)=acos(cos(elevation(i))*sin(azimuth(i)));
    az_1(i)=acos(cos(elevation(i))*cos(azimuth(i))/sin(elev_1(i)));
    az_1_2(i)=asin(sin(elevation(i))/sin(elev_1(i)));
    az_1_3(i)=atan(sin(elevation(i))/(cos(elevation(i))*cos(azimuth(i))));

    angs_1 = [rad2deg(elevation_1(i)), rad2deg(azimuth_1(i)), test_1(elevation_1(i), azimuth_1(i));...
              rad2deg(elev_1(i)),rad2deg(az_1(i)), test_1(elev_1(i), az_1(i));...
              rad2deg(elev_1(i)),rad2deg(az_1_2(i)), test_1(elev_1(i), az_1_2(i));...
              rad2deg(elev_1(i)),rad2deg(az_1_3(i)), test_1(elev_1(i), az_1_3(i))];

    plotGimbal(alpha1(i), alpha2(i), gamma, 1);
    
    az=asin(sin(alpha1(i) + acos((cos(gamma)*sin(alpha2(i)/2))/ sqrt(1 - sin(alpha2(i)/2)^2*sin(gamma)^2)))*sin(2*asin(sin(alpha2(i)/2)*sin(gamma))));
    elev=asin(cos(2*asin(sin(alpha2(i)/2)*sin(gamma)))/sqrt(1 - sin(alpha1(i) + acos((cos(gamma)*sin(alpha2(i)/2))/sqrt(1 - sin(alpha2(i)/2)^2*sin(gamma)^2)))^2*sin(2*asin(sin(alpha2(i)/2)*sin(gamma)))^2));

    disp (sprintf('elevation %f , azimuth %f , alpha1 %f , alpha2 %f',elevation(i),azimuth(i),alpha1(i),alpha2(i)) );
    
    break;
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