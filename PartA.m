
function PartA(pos)
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
    
    for i = 1 : 6
        links(i) = Link('revolute', 'd', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', 0);
        jointTransforms{i} = joint_transform(jointPos(i),a(i),d(i),alpha(i));
        
        prevOriginTransform = eye(4,4);

        if i > 1
            prevOriginTransform = originTransforms{i-1};
        end

        originTransforms{i} = prevOriginTransform * jointTransforms{i};

        disp("Joint Transform " + (i-1) + "->" + i);
        disp(jointTransforms{i});
        disp("Origin Transform 0->" + i);
        disp(originTransforms{i});
    end
    
    ur5 = SerialLink(links, 'name', 'UR5e');
    
    disp("RVC Toolbox Forward Kinematic Solution:")
    fKine = ur5.fkine(jointPos);
    
    disp(fKine);
end

function trans = joint_transform(theta, a, d, alpha)
    trans = trotz(theta) * transl([0 0 d]) * transl([a 0 0]) * trotx(alpha);
end