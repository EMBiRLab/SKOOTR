%inputs
x = 200;
y = -105;
roll = 0;
pitch = 0;
%%%%

ornt = 1;
L0 = 126;
L1 = 136;
L2 = 265;
d = 97;
Ln = sqrt(L0^2 + d^2);
phi_mag = acosd(cosd(pitch)*cosd(roll)/(sind(pitch)^2+(cosd(pitch)^2)*(sind(roll)^2)+(cosd(pitch)^2)*(cosd(roll)^2))^(1/2));
if roll>=0
    phi = 90 - phi_mag;
else
    phi = 90 + phi_mag;
end
psi_dif = atan2d(d,L0);
psi = phi - psi_dif;
x1 = x - Ln*cosd(psi);
y1 = y - Ln*sind(psi);

q2 = ornt*acosd((x1^2 + y1^2 - L1^2 - L2^2)/(2*L1*L2));
q1 = atan2d(y1,x1) + atan2d((L2*sind(q2)),(L1+L2*cosd(q2)));

l0endx = L0*cosd(phi);
l0endy = L0*sind(phi);

dendx = Ln*cosd(psi);
dendy = Ln*sind(psi);

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


xlim([-300 300]);
ylim([-300 300]);


q1
q2
