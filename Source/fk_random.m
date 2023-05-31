
close all
clear             
global uLINK       

%% Initial Values
q_d = [0, 0, 0, 0  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0349, 0, 0, 0, 0, 0.0349, 0];
i=1;
SetupBipedRobot2;  

rand('state',0); 

figure
% while 1
%     %random
%     qR1 = 2/3*pi*(rand(6,1)-0.5);  %  -pi/2 < q < pi/2
%     qR1(4) = pi*rand;          %   0 < q4 < pi 
%     
%     qL1 = pi*(rand(6,1)-0.5);  %  -pi/2 < q < pi/2
%     qL1(4) = pi*rand;          %   0 < q4 < pi 
%     
%     for n=0:5
%         uLINK(RLEG_J0+n).q = qR1(n+1);
%         uLINK(LLEG_J0+n).q = qL1(n+1);
%     end
%     walk
    uLINK(RLEG_J0).q = 0;
    uLINK(RLEG_J1).q = 0;
    uLINK(RLEG_J2).q = -30*ToRad;
    uLINK(RLEG_J3).q = 30*ToRad; 
    uLINK(RLEG_J4).q = 0*ToRad; 
    uLINK(RLEG_J5).q = 0; 
    
    uLINK(LLEG_J0).q = 0;
    uLINK(LLEG_J1).q = 0;
    uLINK(LLEG_J2).q = 0;
    uLINK(LLEG_J3).q = 30*ToRad;
    uLINK(LLEG_J4).q = -30*ToRad;
    uLINK(LLEG_J5).q = 0;
    
    
    uLINK(LARM_J0).q = 15*ToRad;
    uLINK(LARM_J1).q = 0*ToRad;
    uLINK(LARM_J2).q = 0;
    uLINK(LARM_J3).q = -0*ToRad;
    uLINK(LARM_J4).q = 0;
    uLINK(LARM_J5).q = 0;
     
    uLINK(RARM_J0).q = -15*ToRad;
    uLINK(RARM_J1).q = -0*ToRad;
    uLINK(RARM_J2).q = 0;
    uLINK(RARM_J3).q = 0*ToRad;
    uLINK(RARM_J4).q = 0;
    uLINK(RARM_J5).q = 0;
%     % sqwart
%     uLINK(RLEG_J0).q = 0;
%     uLINK(RLEG_J1).q = 0;
%     uLINK(RLEG_J2).q = -60*ToRad;
%     uLINK(RLEG_J3).q = 120*ToRad; 
%     uLINK(RLEG_J4).q = -60*ToRad; 
%     uLINK(RLEG_J5).q = 0; 
%     
%     uLINK(LLEG_J0).q = 0;
%     uLINK(LLEG_J1).q = 0;
%     uLINK(LLEG_J2).q = -90*ToRad;
%     uLINK(LLEG_J3).q = 0*ToRad;
%     uLINK(LLEG_J4).q = 70*ToRad;
%     uLINK(LLEG_J5).q = 0;
%     
%     
%     uLINK(LARM_J0).q = -90*ToRad;
%     uLINK(LARM_J1).q = 0*ToRad;
%     uLINK(LARM_J2).q = 0;
%     uLINK(LARM_J3).q = -0*ToRad;
%     uLINK(LARM_J4).q = 0;
%     uLINK(LARM_J5).q = 0;
%      
%     uLINK(RARM_J0).q = -90*ToRad;
%     uLINK(RARM_J1).q = -0*ToRad;
%     uLINK(RARM_J2).q = 0;
%     uLINK(RARM_J3).q = 0*ToRad;
%     uLINK(RARM_J4).q = 0;
%     uLINK(RARM_J5).q = 0;
%     % fight
%     uLINK(RLEG_J0).q = 0;
%     uLINK(RLEG_J1).q = -10*ToRad;
%     uLINK(RLEG_J2).q = -20*ToRad;
%     uLINK(RLEG_J3).q = 40*ToRad; 
%     uLINK(RLEG_J4).q = -20*ToRad; 
%     uLINK(RLEG_J5).q = 10*ToRad; 
%     
%     uLINK(LLEG_J0).q = 0;
%     uLINK(LLEG_J1).q = 10*ToRad;
%     uLINK(LLEG_J2).q = -20*ToRad;
%     uLINK(LLEG_J3).q = 40*ToRad;
%     uLINK(LLEG_J4).q = -20*ToRad;
%     uLINK(LLEG_J5).q = -10*ToRad;
%     
%     
%     uLINK(LARM_J0).q = -60*ToRad;
%     uLINK(LARM_J1).q = 20*ToRad;
%     uLINK(LARM_J2).q = 60*ToRad;
%     uLINK(LARM_J3).q = -80*ToRad;
%     uLINK(LARM_J4).q = 0;
%     uLINK(LARM_J5).q = 0;
%      
%     uLINK(RARM_J0).q = 30*ToRad;
%     uLINK(RARM_J1).q = -20*ToRad;
%     uLINK(RARM_J2).q = 0*ToRad;
%     uLINK(RARM_J3).q = 80*ToRad;
%     uLINK(RARM_J4).q = 0;
%     uLINK(RARM_J5).q = 50*ToRad;
    uLINK(BODY).p = [0.0, 0.0, 0.3]';
    uLINK(BODY).R = eye(3);
    ForwardKinematics(1);
    
    clf
    DrawAllJoints(1);
    view(38,14)
    axis equal
    zlim([0 0.7])
    xlim([-0.6 0.6])
    ylim([-0.6 0.6])
    grid on
%     
%     fprintf('Ctrl-C to abort')
%     pause
% end
