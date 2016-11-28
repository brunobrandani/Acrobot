%% Simular sistema não-linear
clear all;
% Parâmetros do sistema
  I1 = 0.083;
  I2 = 0.33;
  m1 = 1;
  m2 = 1;
  l1 = 1;
  lc1 = 0.5;
  lc2 = 1;
  g = 9.81;

% Condições iniciais 
%theta 1 em relação a vertical, com 0 para baixo 
%theta 2 em relação a barra 1
theta1 = 0.0;
theta2 = 0.1;

%theta1 = 1.0;
%theta2 = 0.0;

% Ganho controlador linear
K = [-240.6146  -96.6277 -103.8448  -49.0924];
    
% Ganho controlador swing up
k1 = 98.1;       %k1*(q2d-q_2)   => q2d = alfa*atan(q_1d);
k2 = 1.5;        %k2*q_2d
alfa = 0.505;    %limite q_2

%k1 = 2.6;      %k1*q_1d*Eerro
%k2 = 17;        %k2*q_2
%alfa = 1.5;      %alfa*q_2d

% Nova planta simulacao
[Tnl,Ynl] = ode23(@(t,y) ode_acrobot_swingup(t,y,K,k1,k2,alfa),[0 53.56],[theta1;theta2;0;0;0]);
max(Ynl(:,1))

% Gráfico ângulos e velocidades
h = figure;
%hold on;
%Ynl(:,2) = mod(Ynl(:,2)+pi,2*pi)-pi;
plot(Tnl,Ynl(:,1:2));
v1_max = max(abs(Ynl(4,:)))*30/pi
movegui(h,'northeast');
% Configurar gráfico
%axis([53 60 -4 1]);
%
legend('theta 1','theta 2');
%
line([53.56 53.56], get(gca, 'ylim'), 'Color',[0 0 0]);
%

target = pi*[1 1];
t = [0 90];
hold on;
plot(t,target);
target = pi*[-1 -1];
hold on;
plot(t,target);
target = [0 0];
hold on;
plot(t,target);

% Energia

Etopo = 2*((m1*lc1 + m2*l1 + m2*lc2)*g);

i = length(Ynl(:,1));

for i = 1:length(Ynl(:,1))
    
    q = [Ynl(i,1), Ynl(i,2),Ynl(i,3), Ynl(i,4)];

    H = [ m1*lc1^2 + m2*lc2^2 + I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q(2)), m2*lc2^2 + I2 + m2*l1*lc2*cos(q(2));
        m2*lc2^2 + I2 + m2*l1*lc2*cos(q(2)), m2*lc2^2 + I2 ];
    
  
    Epotencial(i) = -(m1*lc1 + m2*l1)*g*cos(q(1)) - m2*g*lc2*cos(q(1)+q(2)) + ((m1*lc1 + m2*l1 + m2*lc2)*g);
    Ecinetica(i) = 1/2*q(3:4)*H*q(3:4)';
    Eatual(i) = Epotencial(i) + Ecinetica(i);
    Eerro(i) = Eatual(i) - Etopo; 
    
    parteA(i) = k1*atan(q(3))*Eerro(i);
    parteB(i) = -k2*q(2);
    parteC(i) = -alfa*q(4);
    u(i) = max(min(parteA(i) + parteB(i) + parteC(i),20),-20);
    
end

h = figure;
plot(Tnl, Ynl(:,3:4));
axis([0 90 -10 10]);
movegui(h,'northwest');

h = figure;
plot(Tnl, Epotencial);
hold on;
plot(Tnl, Ecinetica);
hold on;
plot(Tnl, Eatual);
hold on;
axis([0 90 -0 50]);
legend('Energia Potencial U','Energia Cinética T','Energia total E');
movegui(h,'south');

h = figure;
plot(Ynl(:,1), Ynl(:,3));
axis([-3.5 3.5 -5 5]);
movegui(h,'north');





%%
Epotencial = -(m1*lc1 + m2*l1)*g*cos(Ynl(:,1)) - m2*g*lc2*cos(Ynl(:,1)+Ynl(:,2));


figure;
plot(Tnl, Epotencial);
hold on;
energiaFinal = 24.525;
target = energiaFinal*[1 1];
t = [0 50];
hold on;
plot(t,target);