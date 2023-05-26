function IT_B = trans_rigida(x,y,theta)
theta_rad = deg2rad(theta);
IR_B = [cos(theta_rad), -sin(theta_rad); sin(theta_rad), cos(theta_rad)];
M = [x;y];
IT_B = [IR_B, M ; 0, 0, 1];
end

