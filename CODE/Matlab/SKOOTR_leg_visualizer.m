% solves for and displays joint positions given inputs of x, y, and the orientation of the control hub (read from IMU on the robot). 
% Horizontal black line represents the ground, and green dot represents end effector
% solid lines represent links
% assumes planar constraint (yaw and pitch values must reflect this for accurate results)

%inputs
x = 220;
y = -100;
yaw = 50;
pitch = 50;
%%%%

ornt = 1; % +1 for elbow up, -1 for elbow down
L0 = 126;
L1 = 136;
L2 = 248; % 248 with friction contact, 265 with rolling contact;
d = 97;
Ln = sqrt(L0^2 + d^2);
q0_mag = acosd((L0^2*cosd(yaw)^2*sind(pitch)^2 + L0^2*sind(yaw)^2)^(1/2)/L0);
if yaw>=0
    q0 = q0_mag;
else
    q0 = 180 - q0_mag;
end
gamma_dif = atan2d(d,L0);
gamma = q0 - gamma_dif;
x1 = x - Ln*cosd(gamma);
y1 = y - Ln*sind(gamma);

q2 = ornt*acosd((x1^2 + y1^2 - L1^2 - L2^2)/(2*L1*L2)); % sign here is flipped to reflect how joint is oriented on robot
% q2 is positive in the clockwise direction
q1 = atan2d(y1,x1) + atan2d((L2*sind(q2)),(L1+L2*cosd(q2))); 
% this is missing terms relative to q1 in the paper because this defines q1 = 0 as making L1 parallel to the ground, 
% whereas in the paper q1 = 0 is defined as making L1 parallel to d
% q1 is positive in the counter-clockwise direction

l0endx = L0*cosd(q0);
l0endy = L0*sind(q0);

dendx = Ln*cosd(gamma);
dendy = Ln*sind(gamma);

l1endx = dendx + L1*cosd(q1);
l1endy = dendy + L1*sind(q1);

l2endx = l1endx-L2*cosd(180-q2+q1);
l2endy = l1endy-L2*sind(180-q2+q1);

hold on;

plot(0,0,'.g','MarkerSize', 50);
plot(x,y,'.g','MarkerSize', 50);
plot([0 0], [-100 0], ':c','LineWidth',10);
plot([0 l0endx], [0 l0endy], ':y','LineWidth',10);
plot([l0endx dendx], [l0endy dendy], ':y','LineWidth',10);
plot([0 dendx], [0 dendy], 'k','LineWidth',10);
plot([dendx l1endx], [dendy l1endy], 'b','LineWidth',10);
plot([l1endx l2endx], [l1endy l2endy], 'r','LineWidth',10);
plot([-300 300],[-100 -100],'k','LineWidth',5);

xlim([-300 400]);
ylim([-300 400]);

gamma
x1
y1
q1
q2