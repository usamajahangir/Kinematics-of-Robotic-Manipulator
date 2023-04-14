%% Forward Kinematics
clc
%%%% Enter Values of Thetas
Q1 = input('Enter Value of Theta 1: ');
Q2 = input('Enter Value of Theta 2: ');
Q3 = input('Enter Value of Theta 3: ');
Q4 = input('Enter Value of Theta 4: ');
Q5 = input('Enter Value of Theta 5: ');

%%%% Converting Theta from degree to Radians 
Q1 = (Q1*pi)/180;
Q2 = (Q2*pi)/180;
Q3 = (Q3*pi)/180;
Q4 = (Q4*pi)/180;
Q5 = (Q5*pi)/180;

%%%% Link Lengths for BCN3D Moveo 
L1 = 0.1536;
L2 = 0.221;
L3 = 0.223;

%%%% Calculations for 11 equations
R1 = cos(Q5)*(cos(Q4)*(cos(Q1)*cos(Q2)*cos(Q3) - cos(Q1)*sin(Q2)*sin(Q3)) + sin(Q1)*sin(Q4)) - sin(Q5)*(cos(Q1)*cos(Q2)*sin(Q3) + cos(Q1)*cos(Q3)*sin(Q2));
R2 = -cos(Q5)*(cos(Q1)*cos(Q2)*sin(Q3)+cos(Q1)*cos(Q3)*sin(Q2))-sin(Q5)*(cos(Q4)*(cos(Q1)*cos(Q2)*cos(Q3)-cos(Q1)*sin(Q2)*sin(Q3))+sin(Q1)*sin(Q4));
R3 = cos(Q4)*sin(Q1) - sin(Q4)*(cos(Q1)*cos(Q2)*cos(Q3) - cos(Q1)*sin(Q2)*sin(Q3));
R4 = cos(Q1)*(L3*sin(Q2 + Q3) + L2*cos(Q2));

R5 = cos(Q5)*(cos(Q4)*(cos(Q2)*cos(Q3)*sin(Q1) - sin(Q1)*sin(Q2)*sin(Q3)) - cos(Q1)*sin(Q4)) - sin(Q5)*(cos(Q2)*sin(Q1)*sin(Q3) + cos(Q3)*sin(Q1)*sin(Q2));
R6 = -sin(Q5)*(cos(Q4)*(cos(Q2)*cos(Q3)*sin(Q1) - sin(Q1)*sin(Q2)*sin(Q3)) - cos(Q1)*sin(Q4)) - cos(Q5)*(cos(Q2)*sin(Q1)*sin(Q3) + cos(Q3)*sin(Q1)*sin(Q2));
R7 = -sin(Q4)*(cos(Q2)*cos(Q3)*sin(Q1) - sin(Q1)*sin(Q2)*sin(Q3)) - cos(Q1)*cos(Q4);
R8 = sin(Q1)*(L3*sin(Q2 + Q3) + L2*cos(Q2));

R9  =  cos(Q2 + Q3)*sin(Q5) + sin(Q2 + Q3)*cos(Q4)*cos(Q5);
R10 =  cos(Q2 + Q3)*cos(Q5) - sin(Q2 + Q3)*cos(Q4)*sin(Q5);
R11 =  -sin(Q2 + Q3)*sin(Q4);
R12 =  L1 - L3*cos(Q2 + Q3) + L2*sin(Q2);

%%%% Making Transformation matrics T04
T05 = [R1, R2, R3, R4
       R5, R6, R7, R8
       R9, R10, R11, R12
       0, 0, 0, 1];
%%%% Display T04 Matrics
disp("The Final Transformation matrics T05 for given thetas is: ");
disp(T05);
disp("");
disp("For Inverse Kinematics use following: ");
disp(R1 + " " + R2 + " " + R3 + " " + R4 + " " + R5 + " " + R6 + " " + R7 + " " + R8 + " " + R9 + " " + R10 + " " + R11 + " " + R12 );


