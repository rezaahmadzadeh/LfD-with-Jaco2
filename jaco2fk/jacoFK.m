function [E,T] = jacoFK(qjaco)
%% this is the forward kinematics for Jaco2 arm
%
% Input: qjaco - joint angles in deg (size: 1,6)
%
% Output: 
%       T = transformation matrix from the end-effector to the base
%       E = pose of the end-effector
%
% Example:
%       [E,T] = jacoFK([261.74 171.01 67.60 152.93 37.12 6.91]);
%
% --------------------------------------------
% Code: Reza Ahmadzadeh IRIM-2016
% Email: reza.ahmadzadeh@gatech.edu
% --------------------------------------------
% q = [10000 pi/2 300 20 10 10];
% q = [180, 270, 90, 180, 180, 0] * pi / 180;
% qjaco = [254.75 178.78 66.98 149.19 30.14 23.33];
% qjaco = [261.74 171.01 67.60 152.93 37.12 6.91];
%
[s1,s2] = size(qjaco);
if s1 ~= 1 || s2~=6
    error('the input vector has to be of size 1x6');
end
q = qjaco + [0 -90 90 0 -180 90];
q(1,1) = q(1,1)*-1;
q = q* pi/180;
T = ForwardKinematics(q);
E = T(1:3,4);
disp('T:');
disp(T);
disp(['End-effector: [' num2str(T(1:3,4)') ']'])
end

function Q = ForwardKinematics(q)
%% this function calculates the transformation along all the links
% theta = [q1 q2 q3 q4 q5 q6];
numJoints = size(q,2);
Q = eye(4);
for ii = 1:numJoints
    A = calculateA(q(ii),ii);
    Q = Q*A;
end
end

function A = calculateA(q,n)
%% this function calculated the transformation matrix for each joint
%
% ----- parameters according to Jaco2 documentations -----
D1 = 0.2755;    % base to elbow
D2 = 0.4100;    % arm length
D3 = 0.2073;    % front arm length
D4 = 0.0741;    % first wrist length
D5 = 0.0741;    % second wrist length
D6 = 0.1600;    % wrist to center of the hand
e2 = 0.0098;    % joint 3-4 lateral offset
aa = 30*pi/180;
ca = cos(aa);
sa = sin(aa);
c2a = cos(2*aa);
s2a = sin(2*aa);
d4b = D3 + sa/s2a*D4;
d5b = (sa/s2a*D4 + sa/s2a*D5);
d6b = (sa/s2a*D5 + D6);
% ----- from D-H table -----
alpha = [pi/2 pi pi/2 2*aa 2*aa pi];
a = [0 D2 0 0 0 0];
d = [D1 0 -e2 -d4b -d5b -d6b];
% ----- calcualte transformation for link n -----
A = [cos(q) -sin(q)*cos(alpha(n)) sin(q)*sin(alpha(n)) a(n) * cos(q)
    sin(q) cos(q)*cos(alpha(n)) -cos(q)*sin(alpha(n)) a(n) * sin(q)
    0 sin(alpha(n)) cos(alpha(n)) d(n)
    0 0 0 1];
end


