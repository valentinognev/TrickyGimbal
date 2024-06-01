clear all

deg2rad=pi/180;
rad2deg=180/pi;

roll = 10*deg2rad;
pitch = 20 * deg2rad;
yaw = 30*deg2rad;

rotX=[1,      0,         0;
      0, cos(roll), -sin(roll);
      0, sin(roll),  cos(roll)];

rotY=[ cos(pitch), 0, sin(pitch);
           0,      1,     0;
      -sin(pitch), 0, cos(pitch)];

rotZ=[cos(yaw), -sin(yaw), 0;
      sin(yaw),  cos(yaw), 0;
         0,         0,     1];

rotMatP=eul2rotm([yaw, pitch, roll],'ZYX');
[eul] = rotm2eul(rotMatP,"ZYX");
rotMatP2=rotZ*rotY*rotX
quatPm=quaternion(rotMatP, 'rotmat', 'point');
quatPe=quaternion([roll, pitch,yaw], 'euler', 'XYZ', 'point');
rotm2eul(quatPm.rotmat('point'))*rad2deg
rotm2eul(rotMatP,'ZYX')*rad2deg

rotMatF=eul2rotm([-yaw,-pitch,-roll],'ZYX')

rotMatF2=rotZ'*rotY'*rotX'
quatFm=quaternion(rotMatF2,'rotmat','frame')
quatFe=quaternion([roll,pitch,yaw],'euler','XYZ','frame')
rotm2eul(quatFm.rotmat('frame'))*rad2deg
rotm2eul(rotMatF,'ZYX')*rad2deg
disp ''

% openExample('fusion/TransformStateToSensorFrameExample')