function [x_PB, y_PB, x_PP, y_PP, theta1a, theta2a, theta3a] = Tri_Solution (x_base, y_base, angle_base, xe, ye, angle_plat)
global r_plat r_base SA L 

% Calculate coordinates of triangle vertices
x_PP = xe-r_plat*cos(degtorad(angle_plat));
y_PP = ye-r_plat*sin(degtorad(angle_plat));
x_PB = x_base-r_base*cos(degtorad(angle_base));
y_PB = y_base-r_base*sin(degtorad(angle_base));

% % Calculate distance to platform vertex relative to base joint
x = x_PP - x_PB;
y = y_PP - y_PB;
cos_theta2 = (x^2+y^2-SA^2-L^2)/(2*SA*L);

% If statement here to see if solution exists

% theta2b = atan2(+ sqrt(1-(theta2_sub^2)) , theta2_sub );

if angle_base > 240
    theta2a = atan2(- sqrt(1-(cos_theta2^2)) , cos_theta2 );
    cos_theta1a = (x*(L*cos(theta2a)+SA)+L*sin(theta2a)*y)/(x^2+y^2);
    theta1a = atan2 ( -sqrt(1-(cos_theta1a)^2) , (cos_theta1a) );
    % theta1b_sub = (x*(L*cos(theta2b)+SA)+L*sin(theta2b)*y)/(x^2+y^2);
    % theta1b = atan2 ( -sqrt(1-(theta1b_sub)^2) , (theta1b_sub) );
elseif angle_base > 120
    theta2a = atan2(+- sqrt(1-(cos_theta2^2)) , cos_theta2 );
    cos_theta1a = (x*(L*cos(theta2a)+SA)+L*sin(theta2a)*y)/(x^2+y^2);
    theta1a = atan2 ( -sqrt(1-(cos_theta1a)^2) , (cos_theta1a) );
else
    theta2a = atan2(- sqrt(1-(cos_theta2^2)) , cos_theta2 );
    cos_theta1a = (x*(L*cos(theta2a)+SA)+L*sin(theta2a)*y)/(x^2+y^2);
    theta1a = atan2 ( +sqrt(1-(cos_theta1a)^2) , (cos_theta1a) );
end
theta3a = degtorad(angle_plat)-(theta1a+theta2a);
% theta3b = degtorad(phi+angle_offset)-(theta1b+theta2b);

radtodeg(theta1a)
radtodeg(theta2a)
radtodeg(theta3a)

end