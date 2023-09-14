syms yaw pitch L0 d % variables used in paper (not on robot)


Tx =  [1 0          0         0; % yaw rotation
       0 cos(yaw) -sin(yaw) 0;
       0 sin(yaw)  cos(yaw) 0;
       0 0          0         1];
 
Ty =  [cos(pitch)  0 sin(pitch) 0; % pitch rotation
       0           1 0          0;
       -sin(pitch) 0 cos(pitch) 0;
       0           0 0          1];

T0 =  [1 0 0 0; % L0 translation
       0 1 0 0;
       0 0 1 L0;
       0 0 0 1];

T_final = Ty*Tx*T0; %transform from center of ball to centroid of 3 joint centers
%disp(T_final);

K = transpose(T_final(1:3, 4));
%disp(K);
K_proj = [K(1:2) 0];
%disp(K_proj);

q0 = acos((K_proj(1)^2 + K_proj(2)^2)^(1/2) / L0);
disp(q0);
% ^ this comes from the angle between the last column of the transform matrix and [0,0,1] (wrt center of ball)
% ^ calculated as acos ( (v1 dot v2) / (mag v1 * mag v2) )