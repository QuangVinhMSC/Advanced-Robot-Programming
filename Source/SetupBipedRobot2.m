%%% SetupBipedRobot2.m

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';
UYZ = [0 -1 1]';

U0 = [0 0 0]';
i=1;
q_d = [0, 0, 0, 0  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0349, 0, 0, 0, 0, 0.0349, 0];

uLINK    = struct('name','BODY' , 'm', 1.9217, 'sister', 0, 'child', 2, 'b',[-0.0026 , 0, -0.085]','a',U0,'q',0);

uLINK(2) = struct('name','RLEG_J0' , 'm', 0.0299317014707200, 'sister', 9, 'child', 3, 'b',[-0.0026 -0.05 0]'   ,'a',[0 1 1]','q',0); % RHipYawPitch3_hd
uLINK(3) = struct('name','RLEG_J1' , 'm', 0.0546505785280000, 'sister', 0, 'child', 4, 'b',[0  0   0]'   ,'a',UX,'q',0); % RHipRoll3_hd
uLINK(4) = struct('name','RLEG_J2' , 'm', 0.1638256188643200, 'sister', 0, 'child', 5, 'b',[0  0   0]'   ,'a',UY,'q',0); % RHipPitch3_hd
uLINK(5) = struct('name','RLEG_J3' , 'm', 0.1225434126224000, 'sister', 0, 'child', 6, 'b',[0  0  -0.1]' ,'a',UY,'q',0); % RKneePitch3_hd
uLINK(6) = struct('name','RLEG_J4' , 'm', 0.0563321347904000, 'sister', 0, 'child', 7, 'b',[0  0  -0.103]' ,'a',UY,'q',0);% RAnklePitch3_hd
uLINK(7) = struct('name','RLEG_J5' , 'm', 0.0420389065600000, 'sister', 0, 'child', 8, 'b',[0  0   0  ]' ,'a',UX,'q',0); % RAnkleRoll3_hd
uLINK(8) = struct('name','RLEG_J6' , 'm', 0.0100000000000000, 'sister', 0, 'child', 0, 'b',[0.0181  0.0056 -0.0123]' ,'a',U0,'q',0); % right foot (center point)

uLINK(9) = struct('name','LLEG_J0' , 'm', 0.0299317014707200, 'sister', 16, 'child',10, 'b',[-0.0026  0.05 0]'   ,'a',UYZ,'q',0);% LHipYawPitch3
uLINK(10)= struct('name','LLEG_J1' , 'm', 0.0546505785280000, 'sister', 0 , 'child',11, 'b',[0  0   0]'   ,'a',UX,'q',0);% LHipRoll3
uLINK(11)= struct('name','LLEG_J2' , 'm', 0.1638256188643200, 'sister', 0 , 'child',12, 'b',[0  0   0]'   ,'a',UY,'q',0);% LHipPitch3
uLINK(12)= struct('name','LLEG_J3' , 'm', 0.1225434126224000, 'sister', 0 , 'child',13, 'b',[0  0  -0.1]' ,'a',UY,'q',0);% LKneePitch3
uLINK(13)= struct('name','LLEG_J4' , 'm', 0.0563321347904000, 'sister', 0 , 'child',14, 'b',[0  0  -0.103]' ,'a',UY,'q',0);% LAnklePitch3
uLINK(14)= struct('name','LLEG_J5' , 'm', 0.0420389065600000, 'sister', 0 , 'child',15, 'b',[0  0   0  ]' ,'a',UX,'q',0);% LAnkleRoll3
uLINK(15)= struct('name','LLEG_J6' , 'm', 0.0100000000000000, 'sister', 0 , 'child',0, 'b',[0.0181  0.0056 -0.0123]' ,'a',U0,'q',0);% left foot (center point)
% 
uLINK(16) = struct('name','LARM_J0' , 'm', 000000000000000000, 'sister', 22, 'child',17 , 'b',[-0.0026  0.0983   0.2 - 0.0178]'   ,'a',UY,'q',0); % LShoulderRoll3_hd
uLINK(17) = struct('name','LARM_J1' , 'm', 000000000000000000, 'sister', 0 , 'child',18 , 'b',[0  0   0]'   ,'a',UX,'q',0); % LShoulderPitch3_hd
uLINK(18) = struct('name','LARM_J2' , 'm', 0.0315459954826240, 'sister', 0 , 'child',19 , 'b',[0  0.015  -0.105]' ,'a',UZ,'q',0); % LElbowYaw3_hd
uLINK(19) = struct('name','LARM_J3' , 'm', 0.0272538231228480, 'sister', 0 , 'child',20 , 'b',[0  0  0]' ,'a',UX,'q',0); %  LElbowRoll3_hd
uLINK(20) = struct('name','LARM_J4' , 'm', 0.0327903471168000, 'sister', 0 , 'child',21 , 'b',[0  0   -0.05595  ]' ,'a',UZ,'q',0); % LWristYaw3_hd
uLINK(21) = struct('name','LARM_J5' , 'm', 0.0100000000000000, 'sister', 0 , 'child',0  , 'b',[0  0   -0.0416 ]' ,'a',U0,'q',0); % Left hand (center point)

uLINK(22) = struct('name','RARM_J0' , 'm',  000000000000000000, 'sister', 28, 'child',23 , 'b',[-0.0026  -0.0983   0.2 - 0.0148]'   ,'a',UY,'q',0); % RShoulderRoll3_hd
uLINK(23) = struct('name','RARM_J1' , 'm',  000000000000000000, 'sister', 0 , 'child',24 , 'b',[0  0   0]'   ,'a',UX,'q',0);  % RShoulderPitch3_hd
uLINK(24) = struct('name','RARM_J2' , 'm',  0.0315459954826240, 'sister', 0 , 'child',25 , 'b',[0  -0.015  -0.105]' ,'a',UZ,'q',0);% RElbowYaw3_hd
uLINK(25) = struct('name','RARM_J3' , 'm',  0.0272538231228480, 'sister', 0 , 'child',26 , 'b',[0  0  0]' ,'a',UX,'q',0); % RElbowRoll3_hd
uLINK(26) = struct('name','RARM_J4' , 'm',  0.0327903471168000, 'sister', 0 , 'child', 27 , 'b',[0  0   -0.05595]' ,'a',UZ,'q',0); % RWristYaw3_hd
uLINK(27) = struct('name','RARM_J5' , 'm',  0.0100000000000000, 'sister', 0 , 'child', 0 , 'b',[0  0   -0.0545]' ,'a',U0,'q',0); % right hand (center point)

uLINK(28) = struct('name','HEAD_J0' , 'm',  0.0252233439360000, 'sister', 0, 'child', 29, 'b',[0 0 0.2]'   ,'a',UZ,'q',0); 
uLINK(29) = struct('name','HEAD_J1' , 'm',  0.25223343936+1.8918, 'sister', 0, 'child', 0, 'b',[0 0 0.04]'   ,'a',UY,'q',0); 

FindMother(1);  


for n=1:length(uLINK)
    uLINK(n).dq     = 0;            % Joint Velocity  [rad/s]
    uLINK(n).ddq    = 0;            % Joint Acceleration [rad/s^2]
    uLINK(n).c      = [0 0 0]';     % Center of Mass(Link Local)   [m]
    uLINK(n).I      = zeros(3,3);   % Moment of Inertia(Link Local) [kg.m^2]
    uLINK(n).Ir     = 0.0;          %  [kg.m^2]
    uLINK(n).gr     = 0.0;          %  [-] Gear Ratio
    uLINK(n).u      = 0.0;          %   [Nm]
end

%% Notes 
% sister    - sister ID
% child     - child ID
% mother    - parend ID
% p         - Position in World Coordinates
% R         - Attitude (Orientation) in World Coordinates
% v         - Linear Velocity in World Coordinates
% w         - Angular Velocity in World Coordinates
% q         - Joint Angle
% dq        - Joint Velocity
% ddq       - Joint Acceleration
% a         - Joint Axis Vector(Relative to Parent)
% b         - Joint Relative Position(Relative to Parent)
% vertex    - Shape(Vertex Information, Link Local)
% face      - Shape(Vertice Information (Point Connections)
% m         - Mass 
% c         - Center of Mass(Link Local)
% I         - Moment of Inertia(Link Local)

for n=1:length(uLINK)
    eval([uLINK(n).name,'=',num2str(n),';']);
end

       % rigid body %
shape = [0.02 0.16 0.2115];     %[m]
com   = [0 0.0 0.1058]';        %
SetupRigidBody(BODY, shape,com);

shape = [0.10 0.075 0.04519];    %  [m]
com   = [0.0491  0 -0.0226]';   % 
SetupRigidBody(RLEG_J5, shape,com); %Right foot
% 
shape = [0.10 0.075 0.04519];    %  [m]
com   = [0.0491  0 -0.0226]';   % 
SetupRigidBody(LLEG_J5, shape,com); %Left foot
%y, x, z
shape = [0.3 0.0175 0.5];     %  [m]0.3 0.0375 0.5
com   = [0 0.01 0.0375]';   % 
SetupRigidBody(LARM_J4, shape,com); %Left Arm
% 
shape = [0.08 0.0375 0.2];     %  [m]
com   = [0 -0.02 -0.15]';   % 
SetupRigidBody(RARM_J4, shape,com); %Right Arm

shape = [0.18 0.08 0.15];     %  [m]
com   = [0  0 0.0572]';    %
SetupRigidBody(HEAD_J1, shape,com); %Head


% % %%%%%%%%% initial joint values    %%%%%%%%%%%%
uLINK(RLEG_J0).q = -q_d(i,1);
uLINK(RLEG_J1).q = q_d(i,2);
uLINK(RLEG_J2).q = q_d(i,3);
uLINK(RLEG_J3).q = q_d(i,4); %knee pitch
uLINK(RLEG_J4).q = q_d(i,5); %ankle pitch, rot y
uLINK(RLEG_J5).q = q_d(i,6); % ankle roll, rot x

uLINK(LLEG_J0).q = -q_d(i,7);
uLINK(LLEG_J1).q = q_d(i,8);
uLINK(LLEG_J2).q = q_d(i,9);
uLINK(LLEG_J3).q = q_d(i,10) + 0.7854 ;
uLINK(LLEG_J4).q = q_d(i,11);
uLINK(LLEG_J5).q = q_d(i,12);


uLINK(LARM_J0).q = q_d(i,13) - deg2rad(90);
uLINK(LARM_J1).q = q_d(i,14);
uLINK(LARM_J2).q = q_d(i,15);
uLINK(LARM_J3).q = q_d(i,16);
uLINK(LARM_J4).q = q_d(i,17);
% uLINK(LARM_J5).q = q_d(i,17);
 
uLINK(RARM_J0).q = q_d(i,18)- deg2rad(90);
uLINK(RARM_J1).q = q_d(i,19);
uLINK(RARM_J2).q = q_d(i,20);
uLINK(RARM_J3).q = q_d(i,21);
uLINK(RARM_J4).q = q_d(i,22);
% uLINK(RARM_J5).q = q_d(i,17);
    
BodyPos = [0 0 0.203+0.04519];
uLINK(BODY).p = BodyPos'; % Position  
uLINK(BODY).R = eye(3);


uLINK(BODY).vo = [0, 0, 0]';
uLINK(BODY).w  = [0, 0, 0]';

uLINK(BODY).dvo = [0, 0, 0]';
uLINK(BODY).dw  = [0, 0, 0]';

u_joint = zeros(length(uLINK),1); 

ForwardKinematics(1);


