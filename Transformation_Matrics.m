%% Code to Get Transformation matrics from DH-Parameters (Modified)
clear all
clc

%% Defining Symbols
syms L1 L2 L3 Q1 Q2 Q3 Q4 Q5
alphaa = [0,90,0,90,-90];    % this is the alpha value for all  the link
a=[0,0,L2,0,0];              % Length of the Link
d=[L1,0,0,L3,0];             %Offset
Q=[Q1,Q2,Q3,Q4,Q5];          % joint angle variation

%% Transformation Matrices
for i = 1:5
switch i
    case 1
       T01= [cos(Q(1,i)),-sin(Q(1,i)),0,a(1,i);sin(Q(1,i)).*cosd(alphaa(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i)),-sind(alphaa(1,i))*d(1, i); sin(Q(1,i)).*sind(alphaa(1,i)), cos(Q(1,i)).*sind(alphaa(1,i)), cosd(alphaa(1,i)), cosd(alphaa(1,i))*d(1, i);0,0,0,1];
    case 2
        T12= [cos(Q(1,i)),-sin(Q(1,i)),0,a(1,i);sin(Q(1,i)).*cosd(alphaa(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i)),-sind(alphaa(1,i))*d(1, i); sin(Q(1,i)).*sind(alphaa(1,i)), cos(Q(1,i)).*sind(alphaa(1,i)), cosd(alphaa(1,i)), cosd(alphaa(1,i))*d(1, i);0,0,0,1];
    case 3
        T23= [cos(Q(1,i)),-sin(Q(1,i)),0,a(1,i);sin(Q(1,i)).*cosd(alphaa(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i)),-sind(alphaa(1,i))*d(1, i); sin(Q(1,i)).*sind(alphaa(1,i)), cos(Q(1,i)).*sind(alphaa(1,i)), cosd(alphaa(1,i)), cosd(alphaa(1,i))*d(1, i);0,0,0,1];
    case 4
        T34= [cos(Q(1,i)),-sin(Q(1,i)),0,a(1,i);sin(Q(1,i)).*cosd(alphaa(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i)),-sind(alphaa(1,i))*d(1, i); sin(Q(1,i)).*sind(alphaa(1,i)), cos(Q(1,i)).*sind(alphaa(1,i)), cosd(alphaa(1,i)), cosd(alphaa(1,i))*d(1, i);0,0,0,1];
    case 5
        T45= [cos(Q(1,i)),-sin(Q(1,i)),0,a(1,i);sin(Q(1,i)).*cosd(alphaa(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i)),-sind(alphaa(1,i))*d(1, i); sin(Q(1,i)).*sind(alphaa(1,i)), cos(Q(1,i)).*sind(alphaa(1,i)), cosd(alphaa(1,i)), cosd(alphaa(1,i))*d(1, i);0,0,0,1];
end
end

%% Transformation matrics of wrist w.r.t End-effector 
T05 = T01*T12*T23*T34*T45;
simplify(T05)
