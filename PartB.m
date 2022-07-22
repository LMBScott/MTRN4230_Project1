
function PartB(pos)
    arguments
        pos (1,1) char = 'a'
    end
    
    jointPos = zeros(1, 6);

    if pos == 'b'
        jointPos = deg2rad([-90, -60, 90, 0, 90, 0]);
    else
        jointPos = deg2rad([ -90, -173, 132, 220, 0, 0]);
    end

    format longg;
    startup_rvc;
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
    a = [0, -0.425, -0.3922, 0, 0, 0];
    alpha = [pi / 2, 0, 0, pi / 2, -pi / 2, 0];
    
    links = Link.empty();
    jointTransforms = cell.empty();
    originTransforms = cell.empty();
    jacobian = zeros(6, 6);
    
    for i = 1 : 6
        links(i) = Link('revolute', 'd', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', 0);
        jointTransforms{i} = joint_transform(jointPos(i),a(i),d(i),alpha(i));
        
        prevOriginTransform = eye(4,4);

        if i > 1
            prevOriginTransform = originTransforms{i-1};
        end

        originTransforms{i} = prevOriginTransform * jointTransforms{i};
    end
    
    T06 = originTransforms{6};
    T00 = zeros(4, 4);
    T00(3, 3) = 1;
    on = T06(1:3, 4);

    for i = 1 : 6
        T0i = T00;

        if i > 1
            T0i = originTransforms{i-1};
        end

        zi = T0i(1:3, 3);
        oi = T0i(1:3, 4);

        disp(zi);
        disp(oi);

        jVi = cross(zi, on - oi);
        jwi = zi;
        
        jacobian(:, i) = [jVi; jwi];
    end

    disp("Calculated Jacobian:");
    disp(jacobian);
    
    ur5 = SerialLink(links, 'name', 'UR5e');
    
    rvcJacobian = ur5.jacob0(jointPos);
    disp("RVC Toolbox Jacobian:");
    disp(rvcJacobian);
    
    qDesired = [1; 0.1; 0.1; 0.1; 0.1; 0.1];
    vInstantaneous = jacobian * qDesired;
    disp("End Effector Velocities for Desired Joint Velocities:");
    disp(vInstantaneous);

    vDesired = [250e-3; 0; 0; 0; 0; 0];

    invJ = pinv(jacobian);

    disp("Inverse Jacobian:");
    disp(invJ);

    qInstantaneous = invJ * vDesired;

    disp("Instantaneous Joint Velocities:");
    disp(qInstantaneous);
end

function trans = joint_transform(theta, a, d, alpha)
    trans = trotz(theta) * transl([0 0 d]) * transl([a 0 0]) * trotx(alpha);
end