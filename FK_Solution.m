function [T01, T12, T23, T34, T45] = FK_Solution (theta1, theta2, theta3, theta4, theta5)
global L1 L2 L3 L4 L5

% Create transformation matrix for each link
[T1] = DH_Transformation (0, pi/2, L1, theta1);
[T2] = DH_Transformation (L2, 0, 0, theta2);
[T3] = DH_Transformation (L3, 0, 0, theta3);
[T4] = DH_Transformation (0, -pi/2, 0, (theta4-pi/2));
[T5] = DH_Transformation (0, 0, L4+L5, theta5);

% Calculate Homogeneous transformation matrix
T01 = T1;
T12 = T1*T2;
T23 = T12*T3;
T34 = T23*T4;
T45 = T34*T5;
end