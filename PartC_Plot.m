function PartC_Plot()

    linearSimData = open("data/sim/linear_data.mat");
    linearActualData = open("data/actual/linear_data.mat");
    jointSimData = open("data/sim/joint_data.mat");
    jointActualData = open("data/actual/joint_data.mat");

    drawJointPositions("Simulated Linear Trajectory Joint Positions", linearSimData.jointPs);
    drawJointVelocities('m', "Simulated Linear Trajectory Joint Velocities", linearSimData.jointVs);
    drawJointAccelerations('m', "Simulated Linear Trajectory Joint Accelerations", linearSimData.jointAs);

    drawJointPositions("Actual Linear Trajectory Joint Positions", linearActualData.jointPs);
    drawJointVelocities('m', "Actual Linear Trajectory Joint Velocities", linearActualData.jointVs);
    drawJointAccelerations('m', "Actual Linear Trajectory Joint Accelerations", linearActualData.jointAs);

    drawPath("Linear Trajectory Tool Path", linearSimData.poses, linearActualData.poses);

    drawJointPositions("Simulated Joint Space Trajectory Joint Positions", jointSimData.jointPs);
    drawJointVelocities('r', "Simulated Joint Space Trajectory Joint Velocities", jointSimData.jointVs);
    drawJointAccelerations('r', "Simulated Joint Space Trajectory Joint Accelerations", jointSimData.jointAs);

    drawJointPositions("Actual Joint Space Trajectory Joint Positions", jointActualData.jointPs);
    drawJointVelocities('r', "Actual Joint Space Trajectory Joint Velocities", jointActualData.jointVs);
    drawJointAccelerations('r', "Actual Joint Space Trajectory Joint Accelerations", jointActualData.jointAs);

    drawPath("Joint Space Trajectory Tool Path", jointSimData.poses, jointActualData.poses);
end

function drawJointPositions(plot_title, jointPositions)
    figure;
    hold on;
    plot(jointPositions(:,1));
    plot(jointPositions(:,2));
    plot(jointPositions(:,3));
    plot(jointPositions(:,4));
    plot(jointPositions(:,5));
    plot(jointPositions(:,6));
    title(plot_title)
    xlabel('Time (s)'); 
    ylabel('Joint angles (radians)');
    legend({'Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3'},'Location','southwest');
    hold off;
end

function drawJointVelocities(units, plot_title, jointVelocities)
    arguments
        units (1,1) char = 'm'
        plot_title (1, :) char = "Joint Velocities"
        jointVelocities (:,6) double = double.empty()
    end
    
    unitString = "m/s";

    if units == 'r'
        unitString = "rad/s";
    end

    figure;
    hold on;
    plot(jointVelocities(:,1));
    plot(jointVelocities(:,2));
    plot(jointVelocities(:,3));
    plot(jointVelocities(:,4));
    plot(jointVelocities(:,5));
    plot(jointVelocities(:,6));
    title(plot_title);
    xlabel('Time (s)'); 
    ylabel("Velocity (" + unitString + ")");
    legend({'Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3'},'Location','southwest');
    hold off;
end

function drawJointAccelerations(units, plot_title, jointAccelerations)
    arguments
        units (1,1) char = 'm'
        plot_title (1, :) char = "Joint Accelerations"
        jointAccelerations (:,6) double = double.empty()
    end
    
    unitString = "m/s^2";

    if units == 'r'
        unitString = "rad/s^2";
    end

    figure;
    hold on;
    plot(jointAccelerations(:,1));
    plot(jointAccelerations(:,2));
    plot(jointAccelerations(:,3));
    plot(jointAccelerations(:,4));
    plot(jointAccelerations(:,5));
    plot(jointAccelerations(:,6));
    title(plot_title);
    xlabel('Time (s)'); 
    ylabel("Acceleration (" + unitString + ")");
    legend({'Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3'},'Location','southwest');
    hold off;
end

function drawPath(plot_title, poses1, poses2)
    figure;
    line(poses1(:,1), poses1(:,2), poses1(:,3), 'Color', 'red');
    line(poses2(:,1), poses2(:,2), poses2(:,3), 'Color', 'blue');
    view(3);
    title(plot_title)
    legend(['URSim path', 'Actual UR5e path'])
    xlabel('x-axis'); 
    ylabel('y-axis');
    zlabel('z-axis');
end
