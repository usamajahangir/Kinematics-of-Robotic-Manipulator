%% Inverse Kinematics 
clear all
clc

%%%% Get desired transformation matrics TBW
TBW = inputdlg('Enter TBW seprate with space:');
TBW_mat = str2num(TBW{1});

%%%% performing inverse kinematics 
% Solution for Theta 1, 2 & 3
syms Q1 Q2 Q3
eqns = [cos(Q1)*(0.223*sin(Q2+Q3)+0.221*cos(Q2)) == TBW_mat(4), sin(Q1)*(0.223*sin(Q2+Q3)+0.221*cos(Q2)) == TBW_mat(8), 0.1536-0.223*cos(Q2+Q3)+0.221*sin(Q2) == TBW_mat(12)];
vars = [Q1 Q2 Q3];
[Th_1,Th_2, Th_3] = solve(eqns,vars);
Th_1 = (double(Th_1))*180/pi;
Th_2 = (double(Th_2))*180/pi;
Th_3 = (double(Th_3))*180/pi;

% Solution for Theta 4
Th_4 = asind(TBW_mat(11)/(-1*(sind(Th_2+Th_3))));

% Solution for Theta 5
Th_5 = acosd((cosd(Th_2+Th_3)*TBW_mat(10)+sind(Th_2+Th_3)*cosd(Th_4)*(TBW_mat(9)))/((cosd(Th_2+Th_3))^2+(sind(Th_2+Th_3)*cosd(Th_4))^2));

%%%% Print Angles
disp("Joint Angles (Degrees) will be: ");
disp("Theta 1 = " + Th_1);
disp("Theta 2 = " + Th_2);
disp("Theta 3 = " + Th_3);
disp("Theta 4 = " + Th_4);
disp("Theta 5 = " + Th_5);
