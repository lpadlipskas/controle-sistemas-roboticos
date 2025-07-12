%% Manipulador para drone submarino
% 3 graus de liberdade - 2 juntas rotativas e 1 junta prismatica

clc; clear; close all;

syms L3
syms q1 d2 q3

% Ordem dos parâmetros: theta d a alpha
DH = [0  0   0  -pi/2 ;
      0  0   0   pi/2 ;  
      0  0   L3    0  ];

% Links
% theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica
L(1) = Link([DH(1,:) 0 0],'standard'); % rotativa
L(2) = Link([DH(2,:) 1 0],'standard'); % prismatica
L(3) = Link([DH(3,:) 0 0],'standard'); % rotativa

% robot
q0 = [q1 d2 q3];
manipsub = SerialLink(L,'name','Manipulador Drone Submarino');
T = manipsub.fkine(q0)
A_01 = L(1).A(q1)
A_12 = L(2).A(d2)
A_23 = L(3).A(q3)

A_03 = A_01*A_12*A_23


%% Manipulador para drone submarino - DH (numérico)

% Ordem dos parâmetros: theta d a alpha
DH = [0  0   0  -pi/2 ;
      0  0   0   pi/2 ;  
      0  0   150    0  ];  

% Links
% theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica
L(1) = Link([DH(1,:) 0 0],'standard'); % rotativa
L(2) = Link([DH(2,:) 1 0],'standard'); % prismatica
L(3) = Link([DH(3,:) 0 0],'standard'); % rotativa


% limites das juntas
L(1).qlim = [90 180]*pi/180; % angulo min/max = 90/180 graus - junta rotativa (valor arbitrario)
L(2).qlim = [200 300];  % comprimento min/max = 200/300mm - junta prismatica (valor arbitrario)
L(3).qlim = [30 150]*pi/180; % +- 60 graus em relação a L2


% robot
q0 = [pi/2 250 pi/2]; % posicao inicial (radianos, polegadas)
manipsub = SerialLink(L,'name','Manipulador Drone Submarino');
manipsub.teach(q0)


%% Verificação da cinemática inversa

qd = [100*pi/180 3 -30*pi/80];
T = manipsub.fkine(qd)
qi = manipsub.ikine(T, 'mask', [1 1 1 0 0 0]) % 3 graus de liberdade


%% Jacobiano 

% jacobiano geometrico p/ configuraçao q0
Jl = manipsub.jacob0(q0,'trans') % jacobiano linear
Ja = manipsub.jacob0(q0,'rot')   % jabociano angular
J = [Jl ; Ja]

% jacobiano analitico p/ q0
JA = manipsub.jacob0(q0,'rpy') % rpy / eul / exp


%% Planejamento de trajetoria

%% junta rotativa q1
% aceleração e velocidade contínuas no ponto de passagem
% duração dos segmentos é a mesma (1 segundo)
% variacao do angulo de 100 a 135 graus com 120 graus no ponto intermediario

Q0 = 100; Qi = 120; Qf = 135;
tf = 1;

a10 = Q0;
a11 = 0;
a12 = (12*Qi-3*Qf-9*Q0)/(4*tf^2);
a13 = (-8*Qi+3*Qf+5*Q0)/(4*tf^3);

a20 = Qi;
a21 = (3*Qf-3*Q0)/(4*tf);
a22 = (-12*Qi+6*Qf+6*Q0)/(4*tf^2);
a23 = (8*Qi-5*Qf-3*Q0)/(4*tf^3);

t1 = 0:0.01:1;
pos1 = a10+a11*t1+a12*t1.^2+a13*t1.^3;
vel1 = a11+2*a12*t1+3*a13*t1.^2;
acc1 = 2*a12+6*a13*t1;

t2 = 1:0.01:2;
pos2 = a20+a21*(t2-1)+a22*(t2-1).^2+a23*(t2-1).^3;
vel2 = a21+2*a22*(t2-1)+3*a23*(t2-1).^2;
acc2 = 2*a22+6*a23*(t2-1);

% Plot das trajetórias
figure(2)
subplot(1,3,1)
plot(t1,pos1,'b'); hold on;
plot(t2,pos2,'b'); hold on;
plot([0 1 2],[100 120 135],'ro'); grid on;
title('Position'); xlabel('t [s]'); ylabel ('\theta [\circ]')
subplot(1,3,2)
plot(t1,vel1,'b'); hold on;
plot(t2,vel2,'b'); grid on;
title('Velocity'); xlabel('t [s]'); ylabel ('\theta vel [\circ/s]')
subplot(1,3,3)
plot(t1,acc1,'b'); hold on;
plot(t2,acc2,'b'); grid on;
title('Acceleration'); xlabel('t [s]'); ylabel ('\theta acc [\circ/s^2]')



%% junta prismatica d2
% aceleração e velocidade contínuas no ponto de passagem
% duração dos segmentos é a mesma (1 segundo)
% variacao do comprimento de 210 a 250mm com 225mm no ponto intermediario

d0 = 210; di = 225; df = 250;
tf = 1;

a10 = d0;
a11 = 0;
a12 = (12*di-3*df-9*d0)/(4*tf^2);
a13 = (-8*di+3*df+5*d0)/(4*tf^3);

a20 = di;
a21 = (3*df-3*d0)/(4*tf);
a22 = (-12*di+6*df+6*d0)/(4*tf^2);
a23 = (8*di-5*df-3*d0)/(4*tf^3);

t1 = 0:0.01:1;
pos1 = a10+a11*t1+a12*t1.^2+a13*t1.^3;
vel1 = a11+2*a12*t1+3*a13*t1.^2;
acc1 = 2*a12+6*a13*t1;

t2 = 1:0.01:2;
pos2 = a20+a21*(t2-1)+a22*(t2-1).^2+a23*(t2-1).^3;
vel2 = a21+2*a22*(t2-1)+3*a23*(t2-1).^2;
acc2 = 2*a22+6*a23*(t2-1);

% Plot das trajetórias
figure(3)
subplot(1,3,1)
plot(t1,pos1,'r'); hold on;
plot(t2,pos2,'r'); hold on;
plot([0 1 2],[210 225 250],'ro'); grid on;
title('Position'); xlabel('t [s]'); ylabel ('d [mm]')
subplot(1,3,2)
plot(t1,vel1,'r'); hold on;
plot(t2,vel2,'r'); grid on;
title('Velocity'); xlabel('t [s]'); ylabel ('d vel [mm/s]')
subplot(1,3,3)
plot(t1,acc1,'r'); hold on;
plot(t2,acc2,'r'); grid on;
title('Acceleration'); xlabel('t [s]'); ylabel ('d acc [mm/s^2]')

%% junta rotativa q3
% aceleração e velocidade contínuas no ponto de passagem
% duração dos segmentos é a mesma (1 segundo)
% variacao do angulo de 90 a 45 graus com 60 graus no ponto intermediario

Q0 = 90; Qi = 60; Qf = 45;
tf = 1;

a10 = Q0;
a11 = 0;
a12 = (12*Qi-3*Qf-9*Q0)/(4*tf^2);
a13 = (-8*Qi+3*Qf+5*Q0)/(4*tf^3);

a20 = Qi;
a21 = (3*Qf-3*Q0)/(4*tf);
a22 = (-12*Qi+6*Qf+6*Q0)/(4*tf^2);
a23 = (8*Qi-5*Qf-3*Q0)/(4*tf^3);

t1 = 0:0.01:1;
pos1 = a10+a11*t1+a12*t1.^2+a13*t1.^3;
vel1 = a11+2*a12*t1+3*a13*t1.^2;
acc1 = 2*a12+6*a13*t1;

t2 = 1:0.01:2;
pos2 = a20+a21*(t2-1)+a22*(t2-1).^2+a23*(t2-1).^3;
vel2 = a21+2*a22*(t2-1)+3*a23*(t2-1).^2;
acc2 = 2*a22+6*a23*(t2-1);

% Plot das trajetórias
figure(2)
subplot(1,3,1)
plot(t1,pos1,'g'); hold on;
plot(t2,pos2,'g'); hold on;
plot([0 1 2],[90 60 45],'ro'); grid on;
title('Position'); xlabel('t [s]'); ylabel ('\theta [\circ]')
subplot(1,3,2)
plot(t1,vel1,'g'); hold on;
plot(t2,vel2,'g'); grid on;
title('Velocity'); xlabel('t [s]'); ylabel ('\theta vel [\circ/s]')
subplot(1,3,3)
plot(t1,acc1,'g'); hold on;
plot(t2,acc2,'g'); grid on;
title('Acceleration'); xlabel('t [s]'); ylabel ('\theta acc [\circ/s^2]')


