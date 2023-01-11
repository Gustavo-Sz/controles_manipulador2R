
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Sistema de Controle por Torque computador + PD               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Observação: Este é o arquivo tcpd.m

function [xdot] = slotine(t, x, traj)
% A função acima recebe dois vetores:
%			- vetor t: contém o tempo inicial e o tempo final.
%			- vetor q: contém os ângulos das juntas.

% Dados do manipulador planar de dois lin

% O comprimento dos links é dado por:
a = [1;1];

% Dados do exemplo 4.2 do livro texto:
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

switch traj
    case 1 % rapida
        qd1= -0.75*cos(pi*t)-0.75;
        qd2= -0.75*cos(1.2*pi*t)+3.75;
        dt1 = 0.75*sin(pi*t)*pi;
        dt2 = 0.75*sin(1.2*pi*t)*1.2*pi;
        
        ddt1 = 0.75*cos(pi*t)*pi^2;
        ddt2 = 0.75*cos(1.2*pi*t)*(1.2*pi)^2;
        
    case 2 % lenta
        qd1= -0.75*cos((pi/2)*t)-0.75;
        qd2= -0.75*cos((1.2/2)*pi*t)+3.75;
        dt1 = 0.75*sin((pi/2)*t)*pi/2;
        dt2 = 0.75*sin((1.2/2)*pi*t)*(1.2/2)*pi;
        
        ddt1 = 0.75*cos((pi/2)*t)*(pi/2)^2;
        ddt2 = 0.75*cos((1.2/2)*pi*t)*((1.2/2)*pi)^2;
        
    case 3
        qd1 = 1;
        qd2 = qd1;
        dt1 = 0;
        dt2 = dt1;
        ddt1 = 0;
        ddt2 = 0;
end

qd = [qd1; qd2];
qddot = [dt1;dt2];
qdddot = [ddt1;ddt2];

% Parâmetros do ganho:
kd= 10*[Il1+Ml1*l1^2+(Kr1^2)*Im1+Il2+Ml2*(a(1)^2+l2^2)+Im2+Mm2*a(1)^2 0;0 Il2 + Ml2^2 + Kr2^2*Im2]; 
lambda=[10 0;0 10]; 

% Cálculo dos parâmetros do modelo do manipulador da página 139:

% A Matriz B é:
b11 = Il1+Ml1*l1^2+(Kr1^2)*Im1+Il2+...
    Ml2*(a(1)^2+l2^2+2*a(1)*l2*cos(x(2)))+Im2+Mm2*a(1)^2;
b12 = Il2 + Ml2*(l2^2 +a(1)*l2*cos(x(2))) + Kr2*Im2;
b22 = Il2 + Ml2^2 + Kr2^2*Im2;
b = [b11 b12; b12 b22];

% com massa modificada
mb11 = Il1+Ml1*l1^2+(Kr1^2)*Im1+Il2+...
    60*(a(1)^2+l2^2+2*a(1)*l2*cos(x(2)))+Im2+Mm2*a(1)^2;
mb12 = Il2 + 60*(l2^2 +a(1)*l2*cos(x(2))) + Kr2*Im2;
mb22 = Il2 + 60^2 + Kr2^2*Im2;
mb = [mb11 mb12; mb12 mb22];

% A Matriz C é:
h = -Ml2*a(1)*l2*sin(x(2));
c11 = h*x(4);
c12 = h*(x(3) + x(4));
c21 = -h*x(3);
c22 = 0;
c = [c11 c12;c21 c22];

%com massa modificada
mh = -60*a(1)*l2*sin(x(2));
mc11 = mh*x(4);
mc12 = mh*(x(3) + x(4));
mc21 = -mh*x(3);
mc22 = 0;
mc = [mc11 mc12;mc21 mc22];

%com massa modificada
mg1 = (Ml1*l1 +Mm2*a(1) + 60*a(1))*g*cos(x(1)) ...
    + 60*l2*g*cos(x(1) + x(2));
mg2 = 60*l2*g*cos(x(1) + x(2));
mg = [mg1;mg2];

% A Matriz G é:
g1 = (Ml1*l1 +Mm2*a(1) + Ml2*a(1))*g*cos(x(1)) ...
    + Ml2*l2*g*cos(x(1) + x(2));
g2 = Ml2*l2*g*cos(x(1) + x(2));
g = [g1;g2];


% A Matriz de estados resultante é:
q    = x(1:2); % q1 e q2 calculados pelo ODE45
qdot = x(3:4); % w1 e w2 calculados pelo ODE45


erro = q-qd;
derro  = qdot-qddot; % theta ~

tau= b*(qdddot - lambda*derro) + c*(qddot - lambda*erro) + g - kd*(derro + lambda*erro);

%binv = inv(b);
mbinv = inv(mb);

%xdot =[ qdot ; -binv*(c*qdot + g - tau)];
% com massa modificada
xdot =[ qdot ; -mbinv*(mc*qdot + mg - tau)];

