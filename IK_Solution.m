function [ theta1 , theta2a , theta3a , theta4a , theta2b , theta3b , theta4b] = IK_Solution (position)
global L1 L2 L3 L4 L5

xe = position(1,1);
ye = position(1,2);
ze = position(1,3);
phi = degtorad(position(1,4));

% Calculate location of joint 4
x4 = sqrt(xe^2 + ye^2)-((L4+L5)*cos(phi));
z4 = ze-((L4+L5)*sin(phi))-L1; 
fprintf ( '\n')
theta1 = (atan2 ( ye , xe ));

% Calculate angles for both instances of arm position
cos_theta3 = (x4^2+z4^2-(L2)^2-(L3)^2)/(2*L2*L3);
theta3a = (atan2 ( - sqrt(1-(cos_theta3^2)) , cos_theta3 ));
theta3b = (atan2 ( + sqrt(1-(cos_theta3^2)) , cos_theta3 ));

cos_theta2 = (x4*(L3*cos(theta3a)+L2)+L3*sin(theta3a)*z4)/(x4^2+z4^2);
theta2a = (atan2 ( +sqrt(1-(cos_theta2)^2) , (cos_theta2) ));
cos_theta2 = (x4*(L3*cos(theta3b)+L2)+L3*sin(theta3b)*z4)/(x4^2+z4^2);
theta2b = (atan2 ( +sqrt(1-(cos_theta2)^2) , (cos_theta2) ));

theta4a = phi-(theta2a+theta3a);
theta4b = phi-(theta2b+theta3b);

end
