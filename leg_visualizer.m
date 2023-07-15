ornt = 1;
x1 = -38;
y1 = -226;
L1 = 136;
L2 = 252;
phi = 90;
psi = 52;

q2 = ornt*acosd((x1^2 + y1^2 - L1^2 - L2^2)/(2*L1*L2));
q1 = atan2d(y1,x1) + atan2d((L2*sind(q2)),(L1+L2*cosd(q2))) - psi;

l1endx = L1*cosd(q1+psi);
l1endy = L1*sind(q1+psi);

l2endx = l1endx-L2*cosd(180-q2+q1+psi);
l2endy = l1endy-L2*sind(180-q2+q1+psi);

hold on;

plot(0,0,'.','MarkerSize', 50);
plot(x1,y1,'.','MarkerSize', 50);
plot([0 l1endx], [0 l1endy], 'b','LineWidth',10);
plot([l1endx l2endx], [l1endy l2endy], 'r','LineWidth',10);
plot([-300 300],[-226 -226],'k','LineWidth',5);


xlim([-300 300]);
ylim([-300 300]);


q1+psi
q2