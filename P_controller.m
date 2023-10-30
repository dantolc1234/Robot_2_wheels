vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
    [~,left_Motor] = vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [~,right_Motor] = vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [~,robotHandle] = vrep.simxGetObjectHandle(clientID,'Robotpose',vrep.simx_opmode_blocking);
    
    [~,orientRB] = vrep.simxGetObjectOrientation(clientID, robotHandle, -1, vrep.simx_opmode_streaming);
    [~,positionRB] = vrep.simxGetObjectPosition(clientID, robotHandle, -1, vrep.simx_opmode_streaming);
    
    while(true)
        goal = [1, 1];
        a = move2goal(clientID,vrep,left_Motor,right_Motor,robotHandle,goal);
    end

end

function [isReached] = move2goal(clientID,vrep,left_Motor,right_Motor,gpsHandle,goal)
d_m2g = 0.05;
a_m2g = 70;

Kp = 300;

R = 0.03;
L = 0.1665;
MAX_SPEED = 5;
%% Get orient and postion
[~,robotPosition] = vrep.simxGetObjectPosition(clientID, gpsHandle, -1, vrep.simx_opmode_buffer);
robotOxy = [robotPosition(1),robotPosition(2)];
[~,robotHeading] = vrep.simxGetObjectOrientation(clientID, gpsHandle, -1, vrep.simx_opmode_buffer);

%% Caculate v and angle
V_m2g = goal - robotOxy;
dist2goal = sqrt(V_m2g(1) ^ 2 + V_m2g(2)^2);

if norm(V_m2g) ~= 0
    V_m2g = a_m2g*(V_m2g/norm(V_m2g));
end

if dist2goal < d_m2g
    isReached = true;
else
    isReached = false;
end

desiredOrientation = atan2(V_m2g(2), V_m2g(1));
errorAngle = desiredOrientation - robotHeading(3);

if (abs(errorAngle) > pi)
    if(errorAngle < 0)
        errorAngle = errorAngle + 2 * pi;
    else
        errorAngle = errorAngle - 2 * pi;
    end
end
%% Caculate P controller
omega = Kp * (errorAngle);

v = norm(V_m2g);
vr = (2 * v + omega * L) / 2 * R;
vl = (2 * v + omega * L) / 2 * R;

if vr > MAX_SPEED
    vr = MAX_SPEED;
elseif vr < -MAX_SPEED
    vr = -MAX_SPEED;
end

if vl > MAX_SPEED
    vl = MAX_SPEED;
elseif vl < -MAX_SPEED
    vl = -MAX_SPEED;
end
%% Set velocity
if isReached == false
    vrep.simxSetJointTargetVelocity(clientID,left_Motor,vl,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,right_Motor,vr,vrep.simx_opmode_oneshot);
else 
    vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_oneshot);
end
end







