%% Main ~ Lynx Motion Arm
%% LOAD VARIABLES (Run First)

clear all
clc

% Control for various elements of program (0 = off, 1 = on)
forwardKinematics = 1;
workspacePlot = 0;
inverseKinematics = 0;
taskExecution = 0;

% Define global variables
global L1 L2 L3 L4 L5 

% Define symbolic variables of angle theta and alpha
syms a alpha d theta; syms T T1 T2 T3 T4 T5;
syms L1 L2 L3 L4 L5; syms theta1 theta2 theta3 theta4 theta5;

% Call DH Transformation function and display general matrix
[DH_T] = DH_Transformation (a, alpha, d, theta);

% Define constant link lengths
L1 = 67; L2 = 120; L3 = 128; L4 = 35; L5 = 55;


%% USER DEFINED PLOT

% Joint angles
theta1 = degtorad(0);  % -90 to 90
theta2 = degtorad(0);  %  0 to 180
theta3 = degtorad(0); % -150 to 0
theta4 = degtorad(0);  % -90 to 90
theta5 = degtorad(0);    % -90 to 90

[T01, T12, T23, T34, T45] = FK_Solution (theta1, theta2, theta3, theta4, theta5);

% Extract x, y and z coordinates from transformation matrices
x = [ 0; T01(1,4); T12(1,4); T23(1,4); T34(1,4); T45(1,4) ];
y = [ 0; T01(2,4); T12(2,4); T23(2,4); T34(2,4); T45(2,4) ]; 
z = [ 0; T01(3,4); T12(3,4); T23(3,4); T34(3,4); T45(3,4) ]; 

% Plot coordinates for the end point of each link
clf
hold on
grid on
daspect ( [1 1 1] ) 
axis ([-350 350 -350 350 0 450] )
view([1,-1,1])
xlabel('x Displacement (mm)')
ylabel('y Displacement (mm)')
zlabel('z Displacement (mm)')
plot3 (x,y,z)
hold off

% Label joints for each link length
text (T01(1,4) ,T01(2,4) ,T01(3,4) , 'L1 ' ) ;
text (T12(1,4) ,T12(2,4) ,T12(3,4) , 'L2 ' ) ;
text (T23(1,4) ,T23(2,4) ,T23(3,4) , 'L3' ) ;
text (T34(1,4) ,T34(2,4) ,T34(3,4) , ' ' ) ;
text (T45(1,4) ,T45(2,4) ,T45(3,4) , 'L4/L5' );

fprintf ('For angles [Theta1 = %.2f, Theta2 = %.2f, Theta3 = %.2f, Theta4 = %.2f, Theta5 = %.2f]', radtodeg(theta1), radtodeg(theta2), radtodeg(theta3), radtodeg(theta4), radtodeg(theta5))
fprintf ('EE at [%d,%d,%d]', round(T45(1,4)), round(T45(2,4)), round(T45(3,4)))


%% WORKSPACE SCATTER PLOT

% Initialise large matrix
x_ws = zeros(1,159200);
y_ws = zeros(1,159200);
z_ws = zeros(1,159200);
for i = -90:5:90
    fprintf('Angle %d', i)
    for j =0:20:180
        for k=-150:15:0
            for l =-90:20:90 
                theta1= (i)*(pi/180);
                theta2= (j)*(pi/180);
                theta3= (k)*(pi/180);
                theta4= (l)*(pi/180);
                % theta5 not considered as it has no bearing on the position of the wrist

                % Create TM for each link at each varying angle
                [T1] = DH_Transformation (0, pi/2, L1, theta1);
                [T2] = DH_Transformation (L2, 0, 0, theta2);
                [T3] = DH_Transformation (L3, 0, 0, theta3);
                [T4] = DH_Transformation (0, -pi/2, 0, (theta4-pi/2));
                [T5] = DH_Transformation (0, 0, L4, theta5);
                % Use of L4 only for position to wrist. Add L5 for centre of end effector
                
                % Calculate homogeneous TM of EE
                T45 = T1*T2*T3*T4*T5;
                x_ws(end+1) = T45(1, 4);
                y_ws(end+1) = T45(2, 4);
                z_ws(end+1) = T45(3, 4);
            end
        end
    end
end
% Setup view and graph plot
clf
hold on
daspect ([1,1,1]) 
axis ([-330 350 -330 350 -200 450] )
view([0,1,0])
scatter3 (x_ws,y_ws,z_ws,10);
hold off


%% INVERSE KINEMATICS

% Declare target position and angle of approach in degrees
xyz_phi = [200, 200, 200, 0]

[theta1 , theta2a , theta3a , theta4a , theta2b , theta3b , theta4b] = IK_Solution (xyz_phi);
fprintf ( 'Solution 1: Theta 1= %.2f Theta 2= %.2f Theta 3= %.2f Theta 4= %.2f \n ', rad2deg(theta1), rad2deg(theta2a), rad2deg(theta3a), rad2deg(theta4a) )
fprintf ( 'Solution 2: theta1= %.2f theta 2= %.2f Theta 3= %.2f Theta 4= %.2f \n ', rad2deg(theta1), rad2deg(theta2b), rad2deg(theta3b), rad2deg(theta4b) )


%% DEFINE TASK

% Define set of cartesian points with angle of entry
point1 = [-100, 100, 300, 45];
point2 = [100, 100, 300, 45];
point3 = [0, 100, 290, 0];
point4 = [0, 100, 120, 0];
point5 = [-100, 100, 120, 0];

% Combine points into one array
coords = [point1; point2; point3; point4; point5]
[row,col] = size(coords);

% Setup Graph
clf
hold on
grid on
daspect ( [1 1 1] ) 
view([1,-1,1])
xlabel('x Displacement (mm)')
ylabel('y Displacement (mm)')
zlabel('z Displacement (mm)')

for i = 1 : row
    % Run IK on points
    [theta1, theta2, theta3, theta4, theta2b, theta3b, theta4b] = IK_Solution (coords(i,:));
    % Run FK on angles from IK
    theta5 = 0;
    [T01, T12, T23, T34, T45] = FK_Solution (theta1, theta2, theta3, theta4, theta5);
    
    % Extract x, y and z coordinates from transformation matrices
    x = [ 0; T01(1,4); T12(1,4); T23(1,4); T34(1,4); T45(1,4) ];
    y = [ 0; T01(2,4); T12(2,4); T23(2,4); T34(2,4); T45(2,4) ]; 
    z = [ 0; T01(3,4); T12(3,4); T23(3,4); T34(3,4); T45(3,4) ]; 
    
    % Store EE position in separate array
    x_EE(1,i) = [T45(1,4)];
    y_EE(1,i) = [T45(2,4)];
    z_EE(1,i) = [T45(3,4)];
    
    % Store angles in separate array
    Angle(i,:) = [theta1 , theta2 , theta3 , theta4, theta5];
    
    % Plot coordinates for the end point of each link
    plot3 (x,y,z)
end 

% Rough illustration of what motion path will look like
plot3 (x_EE,y_EE,z_EE,'k')
text (x_EE(1,1) ,y_EE(1,1) ,z_EE(1,1) , '1 ' );
text (x_EE(1,2) ,y_EE(1,2) ,z_EE(1,2) , '2 ' );
text (x_EE(1,3) ,y_EE(1,3) ,z_EE(1,3) , '3 ' );
text (x_EE(1,4) ,y_EE(1,4) ,z_EE(1,4) , '4 ' );
text (x_EE(1,5) ,y_EE(1,5) ,z_EE(1,5) , '5 ' );
hold off

% Display angles for each joint at each stage
radtodeg(Angle)


%% TASK EXECUTION - FREE MOTION

% Setup Graph
clf
hold on
grid on
daspect ( [1 1 1] ) 
axis ([-200 200 -200 200 0 400] )
view(20,11)
xlabel('x Displacement (mm)')
ylabel('y Displacement (mm)')
zlabel('z Displacement (mm)')
curve = animatedline('LineWidth',1.5);
for t = 0:.1:4
    % Run FK for each joint angle determined linear slope equation between
    % each set of points
    if t <= 1
        theta1 = degtorad(-90*t+135);
        theta2 = degtorad(108.27);	
        theta3 = degtorad(-82.6144);
        theta4 = degtorad(19.3513);
        theta5 = degtorad(0);
    
    elseif t <= 2
        theta1 = degtorad(45*t);
        theta2 = degtorad(5.9*t+102.37);
        theta3 = degtorad(30.93*t-113.5416);
        theta4 = degtorad(-81.84*t+101.1865);
        theta5 = degtorad(0);
        
    elseif t <= 3
        theta1 = degtorad(90);
        theta2 = degtorad(51.05*t+12.0754);
        theta3 = degtorad(-103.46*t+155.2324);	
        theta4 = degtorad(52.41*t-167.3113);
        theta5 = degtorad(0);
        
    elseif t <= 4
        theta1 = degtorad(45*t-45);	
        theta2 = degtorad(-40.64*t+287.1355);
        theta3 = degtorad(9.6*t-183.953);	
        theta4 = degtorad(31.04*t-103.1824);
        theta5 = degtorad(0);
    end
    
    % Uncomment to run FK for each joint angle determined by polynomial equation
%     theta1 = degtorad(11.25*t^4 - 97.5*t^3 + 281.25*t^2 - 285*t + 135);
%     theta2 = degtorad(-7.3367*t^4 + 50.562*t^3 - 97.378*t^2 + 54.153*t + 108.27);
%     theta3 = degtorad(17.198*t^4 - 130.74*t^3 + 287.3*t^2 - 173.76*t - 82.614);
%     theta4 = degtorad(-15.488*t^4 + 128.94*t^3 - 319.33*t^2 + 205.87*t + 19.351);
%     theta5 = degtorad(0);
    
    [T01, T12, T23, T34, T45] = FK_Solution (theta1, theta2, theta3, theta4, theta5);
    
    % Extract x, y and z coordinates from transformation matrices
    xe = [ 0; T01(1,4); T12(1,4); T23(1,4); T34(1,4); T45(1,4) ];
    ye = [ 0; T01(2,4); T12(2,4); T23(2,4); T34(2,4); T45(2,4) ]; 
    ze = [ 0; T01(3,4); T12(3,4); T23(3,4); T34(3,4); T45(3,4) ]; 
    
    % Label each point
    if rem(t,1) == 0
        text (T45(1,4),T45(2,4),T45(3,4), num2str(t+1),'Color','red','FontSize',10 );
    end
    % Plot links for arm
    arm = plot3 (xe,ye,ze,'b');
    % Plot coordinates for the end point of each link
    addpoints(curve, T45(1,4),T45(2,4),T45(3,4));
    drawnow
    pause(0.2)
    delete(arm);
end 
hold off

%% TASK EXECUTION - STRAIGHT LINE MOTION

% Setup Graph
clf
hold on
grid on
daspect ( [1 1 1] ) 
axis ([-200 200 -200 200 0 400] )
view(20,11)
xlabel('x Displacement (mm)')
ylabel('y Displacement (mm)')
zlabel('z Displacement (mm)')
curve = animatedline('LineWidth',1.5);

% Set value for theta 5
theta5 = 0;

for t = 0:.1:4
    % Run FK for each joint angle determined linear slope equation between
    % each set of points
    if t <= 1
        x = 200*t-100; y = 100; z = 300; phi = 0;  
    elseif t <= 2
        x = -100*t + 200; y = 100; z = -10*t+310; phi = 0; 
    elseif t <= 3
        x =	0; y = 100; z = -170*t+630; phi = 0;
    elseif t <= 4
        x = -100*t+300; y = 100; z = 120; phi = 0;
    end
    
    % After coordinates are calculated, pass these values to the IK function
    coords = [x, y, z, phi];
    [theta1, theta2, theta3, theta4, theta2b, theta3b, theta4b] = IK_Solution (coords);
    [T01, T12, T23, T34, T45] = FK_Solution (theta1, theta2, theta3, theta4, theta5);
    
    % Extract x, y and z coordinates from transformation matrices
    xe = [ 0; T01(1,4); T12(1,4); T23(1,4); T34(1,4); T45(1,4) ];
    ye = [ 0; T01(2,4); T12(2,4); T23(2,4); T34(2,4); T45(2,4) ]; 
    ze = [ 0; T01(3,4); T12(3,4); T23(3,4); T34(3,4); T45(3,4) ]; 
    
    % Label each point
    if rem(t,1) == 0
        text (T45(1,4),T45(2,4),T45(3,4), num2str(t+1),'Color','red','FontSize',12 );
    end
    
    % Plot links for arm
    arm = plot3 (xe,ye,ze,'b');
    % Plot coordinates for the end point of each link
    addpoints(curve, T45(1,4),T45(2,4),T45(3,4));
    drawnow
    pause(0.2)
    delete(arm);
end 
hold off


%% OBSTACLE AVOIDANCE

% Setup Graph
clf
hold on
grid on
daspect ( [1 1 1] ) 
axis ([-200 200 -200 200 0 400] )
view(68,1)
xlabel('x Displacement (mm)')
ylabel('y Displacement (mm)')
zlabel('z Displacement (mm)')
curve = animatedline('LineWidth',1.5);

% Define Cylinder Paramaters
xf = 20; yf = xf; zf = 50;
x0 = 0; y0 = 100; z0 = 175;
[xCyl, yCyl, zCyl] = cylinder;
h=surf(xCyl*xf+x0,yCyl*yf+y0,zCyl*zf+z0);

%Calculate bounding area of cylinder for Z axis with padding
z_min = z0 - 10;
z_max = z0 + zf + 20;

% Set value for theta 5 and arm step count
theta5 = 0; step_count = 0; arm_returned = false;

for t = 0:.1:4
    % Run FK for each joint angle determined linear slope equation between
    % each set of points
    if t <= 1
        x = 200*t-100; y = 100; z = 300; phi = 0;  
    elseif t <= 2
        x = -100*t + 200; y = 100; z = -10*t+310; phi = 0; 
    elseif t <= 3
        x =	0; % y value will be determined by while loop
        z = -170*t+630; phi = 0;
    elseif t <= 4
        x = -100*t+300;	y = 100; z = 120; phi = 0;
    end
   
    % Determine whether point is within cylinder boundary
    if z > z_min && z < z_max
        % Determine whether point lies within cross sectional area of
        % circle
        if (x-x0)^2+(y-y0)^2 <= xf^2
            while (x-x0)^2+(y-y0)^2 <= xf^2
            fprintf('Obstacle Collision')
            y = y - 5;
            step_count = step_count + 1;
            coords = [x, y, z, phi];
            [theta1, theta2, theta3, theta4, theta2b, theta3b, theta4b] = IK_Solution (coords);
            [T01, T12, T23, T34, T45] = FK_Solution (theta1, theta2, theta3, theta4, theta5);

            % Extract x, y and z coordinates from transformation matrices
            xe = [ 0; T01(1,4); T12(1,4); T23(1,4); T34(1,4); T45(1,4) ];
            ye = [ 0; T01(2,4); T12(2,4); T23(2,4); T34(2,4); T45(2,4) ]; 
            ze = [ 0; T01(3,4); T12(3,4); T23(3,4); T34(3,4); T45(3,4) ]; 

            % Plot links for arm
            arm = plot3 (xe,ye,ze,'b');
            % Plot coordinates for the end point of each link
            addpoints(curve, T45(1,4),T45(2,4),T45(3,4));
            drawnow
            pause(0.2)
            delete(arm);
            end
            % Once arm has been taken out of circle skip to next iteration of
            % for loop
            continue
        end
    end    
    
    % Determine whether obstacle has been cleared
    if z < z_min 
        if arm_returned == 0
            arm_returned = true;
            for i = 0:1:step_count
                y = y + 5;
                coords = [x, y, z, phi];
                [theta1, theta2, theta3, theta4, theta2b, theta3b, theta4b] = IK_Solution (coords);
                [T01, T12, T23, T34, T45] = FK_Solution (theta1, theta2, theta3, theta4, theta5);

                % Extract x, y and z coordinates from transformation matrices
                xe = [ 0; T01(1,4); T12(1,4); T23(1,4); T34(1,4); T45(1,4) ];
                ye = [ 0; T01(2,4); T12(2,4); T23(2,4); T34(2,4); T45(2,4) ]; 
                ze = [ 0; T01(3,4); T12(3,4); T23(3,4); T34(3,4); T45(3,4) ]; 

                % Plot links for arm
                arm = plot3 (xe,ye,ze,'b');
                % Plot coordinates for the end point of each link
                addpoints(curve, T45(1,4),T45(2,4),T45(3,4));
                drawnow
                pause(0.2)
                delete(arm);
            end
        end
    end   
        
    % After coordinates are calculated, pass these values to the IK function
    coords = [x, y, z, phi];
    [theta1, theta2, theta3, theta4, theta2b, theta3b, theta4b] = IK_Solution (coords);
    [T01, T12, T23, T34, T45] = FK_Solution (theta1, theta2, theta3, theta4, theta5);
    
    % Extract x, y and z coordinates from transformation matrices
    xe = [ 0; T01(1,4); T12(1,4); T23(1,4); T34(1,4); T45(1,4) ];
    ye = [ 0; T01(2,4); T12(2,4); T23(2,4); T34(2,4); T45(2,4) ]; 
    ze = [ 0; T01(3,4); T12(3,4); T23(3,4); T34(3,4); T45(3,4) ]; 
    
    % Label each point
    if rem(t,1) == 0
        text (T45(1,4),T45(2,4),T45(3,4), num2str(t+1),'Color','red','FontSize',12 );
    end
    
    % Plot links for arm
    arm = plot3 (xe,ye,ze,'b');
    % Plot coordinates for the end point of each link
    addpoints(curve, T45(1,4),T45(2,4),T45(3,4));
    drawnow
    pause(0.2)
    delete(arm);
end 
hold off