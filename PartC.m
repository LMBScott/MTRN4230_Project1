
function PartC(isSim, moveType)
    arguments
        isSim logical = 1
        moveType (1,1) char = 'l'
    end

    % Given poses, elements 1-3 in mm and 4-6 in rad
    pose1 = [-588.53, -133.30, 100, 2.2214, -2.2214, 0.00];
    pose2 = [-688.53, -133.30, 100, 2.2214, -2.2214, 0.00];
    pose3 = [-688.53, -233.30, 100, 2.2214, -2.2214, 0.00];
    pose4 = [-588.53, -233.30, 100, 2.2214, -2.2214, 0.00];
    
    host = '';

    if isSim
        host = '127.0.0.1';
    else
        host = '192.168.0.100';
    end

    port = 30003;
    ur5 = rtde(host, port);
    
    poses = [];
    jointPs = [];
    jointVs = [];
    jointAs = [];
    jointTs = [];

    dataFileName = "";

    switch moveType
        case 'l'
            dataFileName = "linear_data.mat";
            moveL(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose1); % Start pose, don't collect data for movement
            [ poses, jointPs, jointVs, jointAs, jointTs ] = moveL(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose2);
            [ poses, jointPs, jointVs, jointAs, jointTs ] = moveL(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose3);
            [ poses, jointPs, jointVs, jointAs, jointTs ] = moveL(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose4);
            [ poses, jointPs, jointVs, jointAs, jointTs ] = moveL(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose1);
        case 'j'
            dataFileName = "joint_data.mat";
            moveJ(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose1); % Start pose, don't collect data for movement
            [ poses, jointPs, jointVs, jointAs, jointTs ] = moveJ(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose2);
            [ poses, jointPs, jointVs, jointAs, jointTs ] = moveJ(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose3);
            [ poses, jointPs, jointVs, jointAs, jointTs ] = moveJ(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose4);
            [ poses, jointPs, jointVs, jointAs, jointTs ] = moveJ(ur5, poses, jointPs, jointVs, jointAs, jointTs, pose1);
        otherwise
            disp("Invalid character input for argument moveType. Must be \'l\' or \'j\'.");
    end
    
    ur5.drawJointPositions(jointPs);
    ur5.drawJointVelocities(jointVs);
    ur5.drawJointAccelerations(jointAs);
    ur5.drawJointTorques(jointTs);
    ur5.drawPath(poses);

    savePath = "";

    if isSim
        savePath = "data/sim/";
    else
        savePath = "data/actual/";
    end

    save(savePath + dataFileName, 'poses', 'jointPs', 'jointVs', 'jointAs', 'jointTs');
end

function [ poses, jointPs, jointVs, jointAs, jointTs ] = moveL(ur5, poses, jointPs, jointVs, jointAs, jointTs, nextPose)
    ACC = 0.1;  % Acceleration, m/s^2
    VEL = 0.05; % Velocity, m/s
    [ pose, pos, vel, acc, trq ] = ur5.movel(nextPose, "pose", ACC, VEL, 0, 0);
    poses = cat(1, poses, pose);
    jointPs = cat(1, jointPs, pos);
    jointVs = cat(1, jointVs, vel);
    jointAs = cat(1, jointAs, acc);
    jointTs = cat(1, jointTs, trq);
end

function [ poses, jointPs, jointVs, jointAs, jointTs ] = moveJ(ur5, poses, jointPs, jointVs, jointAs, jointTs, nextPose)
    ACC = 0.1;  % Acceleration, m/s^2
    VEL = 0.05; % Velocity, m/s
    [ pose, pos, vel, acc, trq ] = ur5.movej(nextPose, "pose", ACC, VEL, 0, 0);
    poses = cat(1, poses, pose);
    jointPs = cat(1, jointPs, pos);
    jointVs = cat(1, jointVs, vel);
    jointAs = cat(1, jointAs, acc);
    jointTs = cat(1, jointTs, trq);
end
