function transMotion

% Define platform pose.
posPlat = [20 -1 0];
orientPlat = quaternion([45 35 20], 'eulerd', 'ZYX', 'frame');%quaternion(1, 0, 0, 0);
velPlat = [0 0 0];
accPlat = [0 0 0];
angvelPlat = [0 0 1];

% Define offset of IMU from platform.
posPlat2IMU = [1 2 3];
orientPlat2IMU = quaternion([45 0 0], 'eulerd', 'ZYX', 'frame');

% Calculate IMU pose.
[posIMU, orientIMU, velIMU, accIMU, angvelIMU] ...
    = transformMotion(posPlat2IMU, orientPlat2IMU, posPlat, orientPlat, velPlat, accPlat, angvelPlat);

fprintf('IMU position is: %.2f %.2f %.2f\n', posIMU);

function [posS, orientS, velS, accS, angvelS] = transformMotion( ...
    posSFromP, orientSFromP, posP, orientP, velP, accP, angvelP)
%TRANSFORMMOTION Convert motion quantities between two frames
%   [posS,orientS,velS,accS,angvelS] = TRANSFORMMOTION(posSFromP,
%   orientSFromP, posP) converts the motion quantity in the Platform-frame,
%   posP, to the motion quantities in the Sensor-frame, posS, orientS,
%   velS, accS, and angvelS, using the translation offset, posSFromP, and
%   rotation offset, orientSFromP.
%
%   [posS,orientS,velS,accS,angvelS] = TRANSFORMMOTION(posSFromP,
%   orientSFromP, posP, orientP) converts the motion quantities in the
%   Platform-frame, posP and orientP, to the motion quantities in the
%   Sensor-frame, posS, orientS, velS, accS, and angvelS, using the
%   translation offset, posSFromP, and rotation offset, orientSFromP.
%
%   [posS,orientS, velS,accS,angvelS] = TRANSFORMMOTION(posSFromP,
%   orientSFromP, posP, orientP, velP) converts the motion quantities in
%   the Platform-frame, posP, orientP, and velP, to the motion quantities
%   in the Sensor-frame, posS, orientS, velS, accS, and angvelS, using the
%   translation offset, posSFromP, and rotation offset, orientSFromP.
%
%   [posS,orientS,velS,accS,angvelS] = TRANSFORMMOTION(posSFromP,
%   orientSFromP, posP, orientP, velP, accP) converts the motion quantities
%   in the Platform-frame, posP, orientP, velP, and accP, to the motion
%   quantities in the Sensor-frame, posS, orientS, velS, accS, and angvelS,
%   using the translation offset, posSFromP, and rotation offset,
%   orientSFromP.
%
%   [posS,orientS,velS,accS,angvelS] = TRANSFORMMOTION(posSFromP,
%   orientSFromP, posP, orientP, velP, accP, angvelP) converts the motion
%   quantities in the Platform-frame, posP, orientP, velP, accP, and
%   angvelP, to the motion quantities in the Sensor-frame, posS, orientS,
%   velS, accS, and angvelS, using the translation offset between frames,
%   posSFromP, and rotation offset between frames, orientSFromP.
%
%   Example:
%
%       % Define platform pose.
%       posPlat = [20 -1 0];
%       orientPlat = quaternion(1, 0, 0, 0);
%       velPlat = [0 0 0];
%       accPlat = [0 0 0];
%       angvelPlat = [0 0 1];
% 
%       % Define offset of IMU from platform.
%       posPlat2IMU = [1 2 3];
%       orientPlat2IMU = quaternion([45 0 0], 'eulerd', 'ZYX', 'frame');
% 
%       % Calculate IMU pose.
%       [posIMU, orientIMU, velIMU, accIMU, angvelIMU] ...
%           = transformMotion(posPlat2IMU, orientPlat2IMU, ...
%           posPlat, orientPlat, velPlat, accPlat, angvelPlat);
% 
%       fprintf('IMU position is: %.2f %.2f %.2f\n', posIMU);
%
%   See also IMUSENSOR, GPSSENSOR, WAYPOINTTRAJECTORY

%   Copyright 2019 The MathWorks, Inc.

%#codegen
orientSFromPQuat = orientSFromP;
orientPQuat = orientP;
N=1;

[posSFromP, orientSFromPQuat, posP, orientPQuat, ...
    velP, accP, angvelP] = resizeInputs(N, posSFromP, orientSFromPQuat, ...
    posP, orientPQuat, velP, accP, angvelP);

posS = posP + rotatepoint(orientPQuat, posSFromP);

orientSQuat = orientPQuat .* orientSFromPQuat;
orientS = orientSQuat;

velS = velP + rotatepoint(orientPQuat, crossProduct(angvelP, posSFromP));

accS = accP + rotatepoint(orientPQuat, crossProduct(angvelP, crossProduct(angvelP, posSFromP)));

angvelS = angvelP;


function C = crossProduct(Ain, Bin)
%CROSSPRODUCT Cross product with limited implicit expansion
%   Ain - 1-by-3 or N-by-3 matrix
%   Bin - 1-by-3 or N-by-3 matrix
%
%   C - cross product of A and B, that are the expanded versions of Ain and
%       Bin, respectively.
numA = size(Ain, 1);
numB = size(Bin, 1);
N = max(numA, numB);
expandMat = ones(N, 3);
A = bsxfun(@times, Ain, expandMat);
B = bsxfun(@times, Bin, expandMat);

C = cross(A, B);


function [posSFromP, orientSFromPQuat, posP, orientPQuat, ...
    velP, accP, angvelP] = resizeInputs(N, posSFromP, orientSFromPQuat, ...
    posP, orientPQuat, velP, accP, angvelP)

resize = @(x) repmat(x, N-(size(x,1)-1), 1);

posSFromP = resize(posSFromP);
orientSFromPQuat = quaternion(resize(compact(orientSFromPQuat)));
posP = resize(posP);
orientPQuat = quaternion(resize(compact(orientPQuat)));
velP = resize(velP);
accP = resize(accP);
angvelP = resize(angvelP);

