vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
    %Set toc do trai phai
    [~,left_Motor] = vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [~,right_Motor] = vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [~,front_sensor] = vrep.simxGetObjectHandle(clientID,'front_prox',vrep.simx_opmode_blocking);
    
    [~,detectionState,detectedPoint,~,~] = vrep.simxReadProximitySensor(clientID,front_sensor, vrep.simx_opmode_streaming);
    [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1.0, vrep.simx_opmode_streaming);
    [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1.0,vrep.simx_opmode_streaming);
while(true)
    [~,detectionState,detectedPoint,~,~] = vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_buffer);
    disp(norm(detectedPoint))
    if(detectionState == 1 && norm(detectedPoint)<0.2)
         [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1.0, vrep.simx_opmode_streaming);
         [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,-1.0,vrep.simx_opmode_streaming);
         pause(1)
         [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1.0, vrep.simx_opmode_streaming);
         [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1.0,vrep.simx_opmode_streaming);
         pause(3)
         [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.0, vrep.simx_opmode_streaming);
         [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.0,vrep.simx_opmode_streaming);
    end
end         
end