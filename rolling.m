vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
    [~,left_Motor] = vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [~,right_Motor] = vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    % R of the circle = 0.75
    [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,2.0, vrep.simx_opmode_streaming);
    [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1.6 ,vrep.simx_opmode_streaming);
end