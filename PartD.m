function PartD()
    arguments
        moveType (1,1) char = 'l'
    end

    posA = deg2rad([ -90, -173, 132, 220, 0, 0]);
    poseC = [100, -127.58, 571.29, -1.571, -0.017, 1.57];

    duration = 5; % Trajectory duration, s

    if (moveType == 'j')
        ur5.movej(posA, "joint", 0, 0, duration, 0);
        ur5.movej(poseC, "pose", 0, 0, duration, 0);
    else
        ur5.movel(posA, "joint", 0, 0, duration, 0);
        ur5.movel(poseC, "pose", 0, 0, duration, 0);
    end
end
