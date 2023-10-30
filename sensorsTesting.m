vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
    
    %% initial variables
    [~,left_Motor] = vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [~,right_Motor] = vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [~,front_sensor] = vrep.simxGetObjectHandle(clientID,'front_prox',vrep.simx_opmode_blocking);
    [~,front_right_sensor] = vrep.simxGetObjectHandle(clientID,'front_right',vrep.simx_opmode_blocking);
    [~,front_left_sensor] = vrep.simxGetObjectHandle(clientID,'front_left',vrep.simx_opmode_blocking);
    [~,rear_right] = vrep.simxGetObjectHandle(clientID,'rear_right',vrep.simx_opmode_blocking);
    [~,rear_left] = vrep.simxGetObjectHandle(clientID,'rear_left',vrep.simx_opmode_blocking);
    % Set Vr    and Vl
    [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1, vrep.simx_opmode_streaming);
    [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1,vrep.simx_opmode_streaming);
    
    sensorHandle = [front_sensor,front_right_sensor,rear_right,front_left_sensor,rear_left];
    for i = 1:length(sensorHandle)
        vrep.simxReadProximitySensor(clientID,sensorHandle(i),vrep.simx_opmode_streaming);
    end
    
%% 
while(true)
    detectionStates = readSensors(clientID,vrep,sensorHandle);
    if sum(detectionStates) == 1 || sum(detectionStates) == 3
        [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.5, vrep.simx_opmode_blocking);
        [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,-0.5,vrep.simx_opmode_blocking);
    elseif sum(detectionStates) == 2
        [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1, vrep.simx_opmode_blocking);
        [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1,vrep.simx_opmode_blocking);
    else
        [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1, vrep.simx_opmode_blocking);
        [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1,vrep.simx_opmode_blocking);
    end
end
end
%% Function reads sensor
function [detectionStates] = readSensors(clientID,vrep,sensorHandle)
% Create matrix 1 row, lenth of sensorHandle column
detectionStates = zeros(1, length(sensorHandle));

% Read sensor one by one and fill value in matrix detectionStates
for i = 1:length(sensorHandle)
    [~, detectionState,~,~,~] = vrep.simxReadProximitySensor(clientID,sensorHandle(i),vrep.simx_opmode_buffer);
    if detectionState == 1
        detectionStates(i) = detectionState;
    end
    disp(detectionStates);
end
end


