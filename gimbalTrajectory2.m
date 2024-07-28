clear all
close all

gamma=30/180*pi;   % [deg]

% elevation - from -30 to 30
% azimuth - from -30 to 30

elevation1=-30; %0.67/pi*180;%20;  % [deg] elevation angle with Z - phi
azimuth1=-30; %-1.23/pi*180;%0;  % [deg] azimuth angle in XY plane - curlphi
elevation2=30;%0.69/pi*180;%30;  % [deg] elevation angle with Z - phi
azimuth2=30;%1.96/pi*180;%70;  % [deg] azimuth angle in XY plane - curlphi

rotElev1=rotx(elevation1);   % input to totx, roty, rotz is in degrees 
rotAz1=roty(azimuth1);

rotElev2=rotx(elevation2);
rotAz2=roty(azimuth2);

rot1Gamma=rotx(rad2deg(gamma));
rrod1=[0,.1,0];
rotAlpha1 = @(alpha1) rotz(rad2deg(alpha1));
rrod2=[0,.1,0];
rotAlpha2 = @(alpha2) rotz(rad2deg(alpha2));
rrodCam=[0,0,.1];
rotCam = rotx(rad2deg(gamma));


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
    curr=rotdtheta*curr;

    elevation(i)=pi/2-acos(dot(curr,[0,1,0]));
    azimuth(i)=atan2(curr(3),curr(1));

    dd=elevation(i);
    alpha2(i)=2*asin(csc(gamma)*sin((1/2)*acos(cos(gamma)*sin(dd))));
    alpha1(i)=pi - acos(sec(gamma)*sec((1/2)*acos(cos(gamma)*sin(dd)))*(csc(gamma)*sin((1/2)*acos(cos(gamma)*sin(dd))) - sin(gamma)*sin((1/2)*acos(cos(gamma)*sin(dd))))) - asin(sin(gamma)/sqrt(1 - cos(gamma)^2*sin(dd)^2));
    
    az=asin(sin(alpha1(i) + acos((cos(gamma)*sin(alpha2(i)/2))/ sqrt(1 - sin(alpha2(i)/2)^2*sin(gamma)^2)))*sin(2*asin(sin(alpha2(i)/2)*sin(gamma))));
    elev=asin(cos(2*asin(sin(alpha2(i)/2)*sin(gamma)))/sqrt(1 - sin(alpha1(i) + acos((cos(gamma)*sin(alpha2(i)/2))/sqrt(1 - sin(alpha2(i)/2)^2*sin(gamma)^2)))^2*sin(2*asin(sin(alpha2(i)/2)*sin(gamma)))^2));

    disp (sprintf('elevation %f , azimuth %f , alpha1 %f , alpha2 %f',elevation(i),azimuth(i),alpha1(i),alpha2(i)) );
    
    figure(1); hold on;    
    rod1tip=rotAlpha1(alpha1(i))*rot1Gamma*rrod1;
    rod1=[[0,0,0];rod1tip];
    plot3(rod1(:,1), rod1(:,2), rod1(:,3));

    rod2tip=(rotAlpha1(alpha1(i))*rot1Gamma)*(rotAlpha2(alpha2(i))*rrod2);
    rod2=[[0,0,0];rod2tip]+[1;1]*rod1tip;
    plot3(rod2(:,1), rod2(:,2), rod2(:,3));

    rodCamtip=(rotAlpha1(alpha1(i))*rot1Gamma)*(rotAlpha2(alpha2(i)))*rotCam*rrodCam;
    rodCam=[[0,0,0];rodCamtip]+[1;1]*rod2tip;
    plot3(rodCam(:,1), rodCam(:,2), rodCam(:,3));
    
    plot3([0,curr(1)],[0,curr(2)],[0,curr(3)],'bo');

    disp ''
end
figure(2);
plot(elevation,'x-'); hold on
plot(azimuth,'x-'); hold on
