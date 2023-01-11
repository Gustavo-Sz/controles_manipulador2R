% Observação: Este é o arquivo mtcpd.m
clear all 
close all
q0= [pi/4  -pi/2 0 0];
traj = 1
[T,x] = ode45(@(t,x) tcpd(t,x,traj),[0 50], q0); 

switch traj
    case 1 % rapida
        qd1= -0.75*cos(pi*T)-0.75;
        qd2= -0.75*cos(1.2*pi*T)+3.75;
        
    case 2 % lenta
        qd1= -0.75*cos((pi/2)*T)-0.75;
        qd2= -0.75*cos((1.2/2)*pi*T)+3.75;
    case 3
        qd1 = 1 + 0*T;
        qd2 = qd1;
end

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
