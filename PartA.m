
function PartA()
    startup_rvc;
    theta = zeros(1, 6);
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
    a = [0, -0.425, -0.3922, 0, 0, 0];
    alpha = [pi / 2, 0, 0, pi / 2, -pi / 2, 0];
    
    links = Link.empty();
    
    for i = 1 : 6
        links(i) = Link('revolute', 'd', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', 0);
    end
    
    ur5 = SerialLink(links, 'name', 'UR5e');
    
    posA = deg2rad([ -90, -173, 132, 220, 0, 0]);
    posB = deg2rad([-90, -60, 90, 0, 90, 0]);
    
    fKineA = ur5.fkine(posA);
    fKineB = ur5.fkine(posB);
    
    disp(fKineA);
    disp(fKineB);
    
    % ur5.teach()
    
    T01=joint_transform(-pi/2,0,0.1625,pi/2);
    T12=joint_transform(-3.019,-0.425,0,0);
    T23=joint_transform(2.304,-0.3922,0,0);
    T34=joint_transform(3.840,0,0.1333,pi/2);
    T45=joint_transform(0,0,0.0997,-pi/2);
    T56=joint_transform(0,0,0.0996,0);
    T06=T01*T12*T23*T34*T45*T56;
end

function trans = joint_transform(theta, a, d, alpha)
    trans = trotz(theta) * transl([0 0 d]) * transl([a 0 0]) * trotx(alpha);
end