vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
    [~,left_Motor] = vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [~,right_Motor] = vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,left_Motor,10,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,right_Motor,10,vrep.simx_opmode_oneshot);
  
end 