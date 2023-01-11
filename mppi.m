clear all
close all
q0= [pi/4 -pi/2 0 0];

l1 = 0.5;
l2 = 0.5;
Ml1 = 50;
Ml2 = 50;  
Il1 = 10;
Il2 = 10;
Kr1 = 100;
Kr2 = 100;
Mm1 = 5;
Mm2 = 5;
Im1 = 0.01;
Im2 = 0.01;
g = 9.8; 
a = [1;1];
bar_m11 = Il1+Ml1*l1^2+(Kr1^2)*Im1+Il2+...
    Ml2*(a(1)^2+l2^2)+Im2+Mm2*a(1)^2;

bar_m22 = Il2 + Ml2^2 + Kr2^2*Im2;


kp = [19.75 0;0 19.75]
kd = [400*bar_m11 0; 0 400*bar_m22]
ki = [4e4*bar_m11 0;0 4e4*bar_m22]

traj = 1;

out = sim("robosimppi", 50);

x = out.q.data(:,1:2);
T = out.q.time;
qd1 = out.qd.data(:,1);
qd2 = out.qd.data(:,2);

figure(1)
subplot(2,1,1)
plot(T,x(:,1),T,qd1)
title('q1')
legend(["real","desejado"])
grid
subplot(2,1,2)
plot(T,x(:,2),T,qd2)
title('q2')
legend(["real","desejado"])
grid;

figure(2)
subplot(2,1,1)
plot(T,abs(x(:,1)-qd1))
title('Erro q1')
grid
subplot(2,1,2)
plot(T,(x(:,2)-qd2))
title('Erro q2')
grid;


