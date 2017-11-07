%% Main ~ parallel planar Robot

clear all
clc

% Define symbolic variables of angle theta and alpla
syms a alpha d theta theta1 theta2;

% Constants
global SA L r_plat r_base
SA = 170; % Length of distal arm 
L = 130; % Length of platroximal arm
r_plat = 130; % Radius of platlatform circumcircle
r_base = 290; % Radius of base circumcircle

% User defined centroid and angle wrtg for platform
x_B = 0;
y_B = 0;
x_C = 0; 
y_C = 0;
phi = 0;

% Clear pravious plot
clf
daspect ( [1 1 1] )

% Obtain angles for vertices of base
ivals = 0:120:240;
ni = length(ivals);
% Initialise arrays for storing triangle vertex coordinates and theta values for each leg joint
T_PB = zeros(3,2);
T_PP = zeros(3,2);
T_theta = zeros(3,3);

% Create base and platform plot
for K = 1 : ni
    angle_base = ivals(K) + 30;
    angle_plat = angle_base + (phi)
    [x_PB, y_PB, x_PP, y_PP, theta1, theta2, theta3] = Tri_Solution (x_B ,y_B, angle_base, x_C, y_C, angle_plat) ;
    T_PB(K,:) = [x_PB y_PB];
    T_PP(K,:) = [x_PP y_PP];
    T_theta(K,:) = [theta1 theta2 theta3];
end

radtodeg(T_theta)

% Plot outputs from matrices
line([T_PB(1:3,1);T_PB(1,1)], [T_PB(1:3,2);T_PB(1,2) ], 'Color', 'b')
line([T_PP(1:3,1);T_PP(1,1)], [T_PP(1:3,2);T_PP(1,2) ], 'Color', 'g')

% Forward Kinematics
for K = 1 : 3
% Create transformation matrix for each link and offset translation matrix (T0)
T0 = [1 0 0 T_PB(K,1);  
      0 1 0 T_PB(K,2); 
      0 0 1 0; 
      0 0 0 1];
[T1] = DH_Transformation (SA, 0, 0, T_theta(K,1));
[T2] = DH_Transformation (L, 0, 0, T_theta(K,2));
[T3] = DH_Transformation (r_plat, 0, 0, T_theta(K,3));
T01 = T0*T1;
T12 = T01*T2;
T13 = T12*T3;

% Extract x and z coordinates from transformation matrices / vertices of base
xlabel('x Displacement (mm)')
ylabel('y Displacement (mm)')
x_plot = [ T_PB(K,1); T01(1,4); T12(1,4); T13(1,4); ];
y_plot = [ T_PB(K,2); T01(2,4); T12(2,4); T13(2,4); ]; 

% Plot coordinates for the end point of each link
line(x_plot,y_plot, 'Color', 'r')

% Label joints for each link length
text (T_PB(K,1) ,T_PB(K,2) , ['PB' num2str(K)]) ;
text (T01(1,4) ,T01(2,4) ,T01(3 ,4) , ['M' num2str(K)]) ;
text (T12(1,4) ,T12(2,4) ,T12(3 ,4) , ['PP' num2str(K)]) ;
text (T13(1,4) ,T13(2,4) ,T13(3 ,4) , ['C']) ;
end


%% WORKSPACE 

% Initialise large matrix
x_ws = zeros(1,15920);
y_ws = zeros(1,15920);
z_ws = zeros(1,15920);

clf
hold on
daspect ([1,1,1]) 
xlabel('x Displacement (mm)')
ylabel('y Displacement (mm)')

for l = 1 : 3

    fprintf('Chain %d', l)
    for i = -180:20:180
        for j =-180:20:180
            for k=-180:20:180
                theta1= degtorad(i);
                theta2= degtorad(j);
                theta3= degtorad(k);

                % Create transformation matrix for each link and offset translation matrix (T0)
                T0 = [1 0 0 T_PB(l,1);  
                      0 1 0 T_PB(l,2); 
                      0 0 1 0; 
                      0 0 0 1];
                [T1] = DH_Transformation (SA, 0, 0, theta1);
                [T2] = DH_Transformation (L, 0, 0, theta2);
                [T3] = DH_Transformation (r_plat, 0, 0, theta3);

                % Calculate homogeneous TM of EE
                T01 = T0*T1;
                T12 = T01*T2;
                T13 = T12*T3;

                % Extract x,y and z coordinates from final transformation
                x_ws(end+1) = T13(1, 4);
                y_ws(end+1) = T13(2, 4);
            end
        end
    end
    scatter (x_ws,y_ws,10);
end
% Plot base
line([T_PB(1:3,1);T_PB(1,1)], [T_PB(1:3,2);T_PB(1,2) ], 'Color', 'b')
hold off
