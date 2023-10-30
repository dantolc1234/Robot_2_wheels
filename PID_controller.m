vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
    [~,left_Motor] = vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [~,right_Motor] = vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [~,gpsHandle] = vrep.simxGetObjectHandle(clientID,'GPS',vrep.simx_opmode_blocking);
    [~,orientHandle] = vrep.simxGetObjectHandle(clientID,'GyroSensor',vrep.simx_opmode_blocking);
    
    [~,positionRB] = vrep.simxGetObjectPosition(clientID,gpsHandle, -1, vrep.simx_opmode_streaming);
    [~,orientRB] = vrep.simxGetObjectOrientation(clientID,orientHandle, -1, vrep.simx_opmode_streaming);
    
    %% main loop 
    goal = [1,1; 2,1; 0,-1; -1,1]; %4 points to go
    preError = 0;
    disp(length(goal));
    for i = 1:length(goal)
        while(true)
            [a, b] = move2goal(clientID,vrep,left_Motor,right_Motor,gpsHandle,orientHandle,goal(i),preError);
            preError = b;
            if a == 1
               pause(1)
               break;
            end
        end
    end
end

%% function return 2 value: one is getTarget or not, the other is the preError
function [isReached,preE] = move2goal(clientID,vrep,left_Motor,right_Motor,gpsHandle,orientHandle,goal,preError)

%% initial valuables
d_m2g = 0.05; %offset target
a_m2g = 70;
dt = 0.05; % 5ms - delta time default of vrep (t)

Kp = 300; % Coefficient
Ki = 30; % Coefficient
Kd = 3; % Coefficient

R = 0.03; % Wheel's Radius (meter)
L = 0.1665; % The length between two wheels (meter)
MAX_SPEED = 100;

%% Get orient and postion
[~,robotPosition] = vrep.simxGetObjectPosition(clientID, gpsHandle, -1, vrep.simx_opmode_buffer);
robotOxy = [robotPosition(1),robotPosition(2)];
[~,robotHeading] = vrep.simxGetObjectOrientation(clientID, orientHandle, -1, vrep.simx_opmode_buffer);

%% Caculate V_m2g
V_m2g = goal - robotOxy;
dist2goal = sqrt(V_m2g(1) ^ 2 + V_m2g(2)^2);
% normalize V_m2g
if norm(V_m2g) ~= 0
    V_m2g = a_m2g*(V_m2g/norm(V_m2g));
end

%% Bool flag
if dist2goal < d_m2g
    isReached = true;
else
    isReached = false;
end

%% Caculate errorAngle
desiredOrientation = atan2(V_m2g(2), V_m2g(1));
errorAngle = desiredOrientation - robotHeading(3);
% Normalize errorAngle
if (abs(errorAngle) > pi)
    if(errorAngle < 0)
        errorAngle = errorAngle + 2 * pi;
    else
        errorAngle = errorAngle - 2 * pi;
    end
end
preE = getPreError(errorAngle); % transfer preError to the next loop
%% Caculate PID controller
Pout = Kp * (errorAngle);
Iout = Ki * (errorAngle + preError) * dt;
Dout = Kd * (errorAngle - preError) / dt;
    
omega = Pout + Iout + Dout;

v = norm(V_m2g);
vr = (2 * v + omega * L) / 2 * R;
vl = (2 * v - omega * L) / 2 * R;

%% limit the velocity of two wheels
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
return;
end

%% Fucntion save the preError for the next loop
function [preError] = getPreError(errorAngle)
    preError = errorAngle;
    return;
end







