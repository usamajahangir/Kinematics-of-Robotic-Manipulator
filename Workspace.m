%d 
d1 = 153.6;
d2 = 0;
d3 = 0;
d4 = 223;
d5 = 0;

%a 
a1 = 0;
a2 = 0;
a3 = 221;
a4 = 0;
a5 = 0;

%Alpha 
al1 = 0;
al2 = pi/2;
al3 = 0;
al4 = pi/2;
al5 = -pi/2;

%DH Parameters


L1=Link([0,d1,a1,al1,0,0], 'modified');  %% theta, d, a, alpha, 0 for revolute, variable(revolute and ) ofset 
L1.qlim = [0 pi/2];
L2=Link([0,d2,a2,al2,0,0], 'modified');
L2.qlim = [0 pi/2];
L3=Link([0,d3,a3,al3,0,0], 'modified');
L3.qlim = [0 pi/2];
L4=Link([0,d4,a4,al4,0,0], 'modified');
L4.qlim = [0 pi/2];
L5=Link([0,d5,a5,al5,0,0], 'modified');
L5.qlim = [0 pi/2];

System = SerialLink([L1 L2 L3 L4 L5], 'name', 'Assistive Feeding System');
System.plot([0,0,0,0,0],'workspace', [-100 400 -100 400 -50 400]); %,'workspace', [-20 40 -20 20 -10 40]
System.teach;

%forward kinematics
q_test = [0.0000,   -0.1604,  -0.9539 ,   0.0000 ,        0];
T=System.fkine (q_test)

%inverse kinematics
T_test = transl(18, 0, 20)* trotx(pi/2)
q0 = [0,0, 0, 0, 0]; % Initial guess for joint angles
q = System.ikine(T_test, q0,'mask', [1 1 1 1 1 0], 'ilimit', 1000)