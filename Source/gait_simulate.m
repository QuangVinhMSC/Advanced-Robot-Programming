close all
clear              % clear work space


%% Initial Values
global uLINK G BodyPos
G = 9.8;  % Gravity acceleration [m/s^2]
i=1;
BodyPos= [0.0, 0.0, 0.203 + 0.04519 ];
q_d = [0, 0, 0, 0  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0349, 0, 0, 0, 0, 0.0349, 0];


SetupBipedRobot2;   % Set the biped robot of Fig.2.19 and Fig.2.20

%%%%%%%%%%% set non singular posture %%%%%%%%%%%%
uLINK(RLEG_J2).q = -0*ToRad;
uLINK(RLEG_J3).q = 0.0*ToRad;
uLINK(RLEG_J4).q = -0.0*ToRad;

uLINK(LLEG_J2).q = -.0*ToRad;
uLINK(LLEG_J3).q = 0.0*ToRad;
uLINK(LLEG_J4).q = -0*ToRad;

% uLINK(BODY).p = [0.0, 0.0, 0.3]';
% uLINK(BODY).R = eye(3);

%%%%%%%%%%% random target foot position and orientation %%%%%%%%%%%%

rand("state",0)

figure
dk=1;
dk2 = 0;
chuki = 0;
uLINK(BODY).p = [0.1, 0.0, 0.15]';
uLINK(BODY).R = eye(3);
Rfoot.p = [0, -0.2, -0.5]';
Rfoot.R = RPY2R([0,0,0*ToRad]');
Lfoot.p = [0, 0.2, -0.5]';
Lfoot.R = RPY2R([0,0,0*ToRad]');
xa = 0;
za = 0;
xc = 0;
zc = 0;
for i = 0:0.8:32*pi
    
    if (dk)
        a = i-2*pi*chuki;
        b = i-4*pi*chuki;
        
        xa = 0.1*(a-sin(a));
        za = 0.1*(1-cos(a));
        
        xb = 0.1*(i-sin(i));
        %zb = 0.6-sqrt(0.6^2-xb^2);
        uLINK(BODY).p = [0.1+xb*0.225, 0.0, 0.15]';
        uLINK(BODY).R = eye(3);
        Rfoot.p = [-0.15+xa-xb*0.275, -0.2, -0.5+za]';
        Lfoot.p = [0.15+xc-xb*0.275, 0.2, -0.5+zc]';
        Rfoot.R = RPY2R([0,0,0*ToRad]'); 
        if b>2*pi
            dk=0;
            dk2=1;
        end
    end
    if (dk2)
        c = i-2*pi*chuki-2*pi;
        d = i-4*pi*chuki-2*pi;
        
        xc = 0.1*(c-sin(c));
        zc = 0.1*(1-cos(c));
        
        xd = 0.1*(i-sin(i));
%         zd =0.6-sqrt(0.6^2-xd^2);
        
        uLINK(BODY).p = [0.1+xd*0.225, 0.0, 0.15]';
        uLINK(BODY).R = eye(3);
        Lfoot.p = [0.15+xc-xd*0.275, 0.2, -0.5+zc]';
        Rfoot.p = [-0.15+xa-xd*0.275, -0.2, -0.5+za]';
        Lfoot.R = RPY2R([0,0,0*ToRad]');
        if d>2*pi
            dk=1;
            dk2=0;
            chuki = chuki+1;
        end
    end 

    Rhand.p = [0.15,-0.08,0.0]';
    Rhand.R = RPY2R([0,0*ToRad,0]');  

    Lhand.p = [0, 0.5, -0.5]';
    Lhand.R = RPY2R([0,20*ToRad,0*ToRad]');  

    
    %%% Analytical inverse kinematics solution
    qR2 = IK_leg(uLINK(BODY),-0.1,0.3,0.3,Rfoot);
    qL2 = IK_leg(uLINK(BODY), 0.1,0.3,0.3,Lfoot);
    qR1 = IK_leg(uLINK(BODY),-0.1,-0.3,0.3,Rhand);
    qL1 = IK_leg(uLINK(BODY),-0.1,-0.3,-0.3,Lhand);
    for n=0:5
        uLINK(RLEG_J0+n).q = qR2(n+1);
        uLINK(LLEG_J0+n).q = qL2(n+1);
        uLINK(RARM_J0+n).q = qR1(n+1);
        uLINK(LARM_J0+n).q = qL1(n+1);
    end

    ForwardKinematics(1);
    
    clf
    DrawAllJoints(1);
    view(75,10)
    axis equal
    zlim([-0.1 0.7])
    xlim([0 3])
    ylim([-0.6 0.6])
    grid on

%     fprintf('Type any key for another pose, Ctrl-C to abort\n');
    pause(0.01)
end
