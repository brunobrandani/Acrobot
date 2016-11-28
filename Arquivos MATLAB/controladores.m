addpath('c:\Users\Bruno\Documents\Graduação\TG\Simulação');
opengl('save', 'software');

%% Hinf
A = [0, 0, 1, 0; 0, 0, 0, 1; 12.6292, -12.6926, 0, 0; -14.7488, 29.6119, 0, 0];
B = [0; 0; -3.0147; 6.0332];
C = [ 1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 0, 0];
D = [0; 0; 0.15];
He = [4.6630 2.3300; 2.3300 1.3300];
Ee = [1 0;0 1];
E = [0 0;0 0;He\Ee];

D = [0; 0; 0.075];
[norma, var] = cont_hinf_norm(A,B,C,D,E);
%Nova iteração
[normab, varb] = cont_hinf_sf(A,B,C,D,E,1.2*norma);
norma
1.2*norma
Khinf1 = -varb.K

%%

alphaD = 12.0;
[norma, var] = cont_hinf_norm(A,B,C,alphaD*D,E);
%Nova iteração
[normab, varb] = cont_hinf_sf(A,B,C,alphaD*D,E,1.2*norma);
Khinf2 = -varb.K

%% LQR
A = [0, 0, 1, 0; 0, 0, 0, 1; 12.6292, -12.6926, 0, 0; -14.7488, 29.6119, 0, 0];
B = [0; 0; -3.0147; 6.0332];
C = [10, -5, 0, 0; -5, 10, 0, 0; 0, 0, 0, 0];
D = [0; 0; 100];
%%
A =  [        0         0    1.0000         0;
         0         0         0    1.0000;
   90.7799 -127.4941         0         0;
 -102.4414  281.9989         0         0];

B =    1.0e+03 * [
         0;
         0;
   -3.8875;
    7.6040];
 

C = [10, -5, 0, 0; -5, 10, 0, 0; 0, 0, 0, 0];
Q = C'*C;

R = 0.1;
[Klqr0,P1,E1] = lqr(A,B,Q,R);

R = 0.5;
[Klqr05,P1,E1] = lqr(A,B,Q,R);

R = 1;
[Klqr1,P1,E1] = lqr(A,B,Q,R);

R = 5;
[Klqr15,P1,E1] = lqr(A,B,Q,R);

R = 10;
[Klqr2,P1,E1] = lqr(A,B,Q,R);

R = 50;
[Klqr25,P1,E1] = lqr(A,B,Q,R);

R = 100;
[Klqr3,P1,E1] = lqr(A,B,Q,R);

%% Simular sistema não-linear
clc
% Condições iniciais
theta1 = pi-0.6;
theta2 = 1.2;
theta1_dot = 0;
theta2_dot = 0;

% Controle
Kteste = [-61.7835  -28.9532   -9.9639   -5.0677];
Kteste = [-418.5 -197.0 -67.5 -34.4];

% Nova planta simulacao
[Tnl,Ynl] = ode23(@(t,y) ode_acrobot_mod_param(t,y,Kteste),[0 1],[theta1;theta2;theta1_dot;theta2_dot]);

% Gráfico ângulos e velocidades
figure;
plot(Tnl,Ynl(:,1:4));
Ynl = (Ynl-repmat([pi 0 0 0],[length(Ynl) 1]));
% Gráfico esforço de controle
u = -Kteste*Ynl';
u = max(min(u,1.2),-1.2);
hold on;
plot(Tnl,u);
axis([0 1 -50 30]);

% Gráfico torque
% figure;
% Kp = 98.1;
% Kd = 1.5;
% u = (Kp*(0.505*atan(Ynl(:,3)) - Ynl(:,2)) - Kd*Ynl(:,4));
% u = max(min(u,1.2),-1.2);
% plot(Tnl,u);
% axis([0 0.1 -1.2 1.2]);
  

%Ynl = (Ynl-repmat([pi 0 0 0],[length(Ynl) 1]));
% Gráfico esforço de controle
%u = -Kteste*Ynl';
%u = max(min(u,1.2),-1.2);
%umax = max(abs(u))               %N.m
%umax_kg_cm = umax*10.19716213    %kg.cm
%wmax = max(abs(Ynl(:,4)))
%vmax = wmax*30/pi  %RPM
%hold on;
%plot(Tnl,u);
%%
% Configurar gráfico
axis([0 1 -35 65]);

legend('theta 1','theta 2','velocidade angular 1','velocidade angular 2','torque');

Ymatlab = Ynl;
%% Teste automatizado - Ponto controlável
% Resposta do teste
clear controle;

K1 = Klqr0;
K2 = Klqr05;
K3 = Klqr1;
K4 = Klqr15;
K5 = Klqr2;
K6 = Klqr25;
K7 = Klqr3;

K_array = [K6];
numeroElementos = 1;

% Limites de angulos para calculos
minTheta1 = -1.8;
minTheta2 = 0.0;
maxTheta1 = 0.25;
maxTheta2 = 4.0;
step = 0.01;

% Vetores para loops
waitbarLimit = round((maxTheta1-minTheta1)/step + 1);
maxLinha = int64((maxTheta1-minTheta1)/step + 1);
maxColuna = int64((maxTheta2-minTheta2)/step + 1); 
X = minTheta1:step:maxTheta1;
Y = minTheta2:step:maxTheta2;
lengthAnswer = maxLinha*maxColuna;

% Contadores para loops
coluna = 0;
Kloop = 0;
offset = Kloop + 1;
h = waitbar(0,'Carregando...');
% Loop para variar ganho K
for Kfor = K_array';

    clear controle;  
    controle = 999999*ones(maxLinha,maxColuna);
    K = Kfor';
    
    umax = 0;
    linha = 0;
    Kloop = Kloop + 1;
    
    % Loop para variar theta1
    for theta1 = X
        porcentagem = ((Kloop-offset)*waitbarLimit + linha)/(numeroElementos*waitbarLimit);
        waitbar(porcentagem,h,sprintf('Teste %d%%', round(100*porcentagem)));
        linha = linha + 1;
        coluna = 0;
        nextLineFlag = 0;
        % Loop para variar theta2
        for theta2 = Y
            coluna = coluna + 1;
            % verificar se linha nao é mais estável (ultimos 15 não
            % estáveis)
            if( (nextLineFlag == 0) || (coluna <= 15) || (min( controle(linha, (coluna-15:coluna-1)) ) ~= 999999) )
                
                % Simular sistema
                [Tnl,Ynl] = ode23(@(t,y) ode_acrobot_mod_param(t,y,K),[0 5],[pi+theta1;theta2;0;0]);
                
                % Tempo em análise - Último ponto do gráfico
                % Calculo da norma para todos os pontos
                i = length(Tnl);
                N = (Ynl(i,1)-pi)^2 + (Ynl(i,2))^2 + (Ynl(i,3))^2 + (Ynl(i,4))^2;
                    
                % Caso nao tenha estabilizado ou pendulo 1 passe de 90º,
                % ponto instavel
                if(N > 1e-3 || (max(abs(abs(Ynl(:,1))-pi)) > 1.6))
                    controle(linha, coluna) = 999999;
                else
                    u = -K*(Ynl-repmat([pi 0 0 0],[length(Ynl) 1]))';
                    u = max(min(u,1.2),-1.2);
                    controle(linha, coluna) = max(abs(u));
                    nextLineFlag = 1;
                end
            else
                % Não estável pelo histórico da linha
                controle(linha, coluna) = 999999;
            end
        end
    end
    
    % Plotar grafico
    figure;
    v = [1.2001];
    contourf(X,Y,controle',v,'ShowText','off');
    grid on;
    title('LQR Ro = 50');
    
end
close(h);
%% Plotar o grafico
figure;
while(f > 0)
    s = scatter(answer(f,1),answer(f,2));
    s.LineWidth = 0.01;
    if(answer(f,4) == 1)
        s.MarkerEdgeColor = 'g';
        if(answer(f,4) < 10)
            green = 1;
        else if(answer(f,4) < 30)
                green = 0.5;
            else
                green = 0;
            end
        end
        s.MarkerFaceColor = [0 green 0];
        s.MarkerEdgeColor = [0 green 0];
    else
        s.MarkerEdgeColor = [1 0 1];
        s.MarkerFaceColor = [1 0 1];
    end
hold on;
f=f-1;
end
axis([-5 5 -5 5]);
%% ----------------------------------------------------------------------------------------------------------------------------
%% ----------------------------------------------------------------------------------------------------------------------------
%% ----------------------------------------------------------------------------------------------------------------------------
%% Simular sistema linear
[Tl,Yl] = ode23(@(t,y) ode_acrobot_linear(t,y,K),[0 5],[0.01;0;0;0]);
figure;
%Yl(:,1) = mod(Yl(:,1),pi);
%Yl(:,2) = mod(Yl(:,2),pi);
plot(Tl,Yl);
u = -K*Yl';
hold on;
plot(Tl,u);
axis([0 5 -3 3]);
legend('theta 1','theta 2','w1','w2','u');

%% Simulação LQR Linear
t = 0:0.001:5;
u = zeros(size(t));
x0 = [0.1, 0, 0, 0];
Cz = [1 0 0 0; 0 1 0 0];
Dz = [0;0;1];
sys = ss(A - B*K, [0;0;0;0], [Cz;-K], [0;0;0]);
[y,t,x] = lsim(sys, u, t, x0);
figure;
plot(t,y(:,1),'r')
hold on;
plot(t,y(:,2),'b')
hold on;
plot(t,y(:,3),'g')
axis([0 5 -20 20]);

%%
%Simulação Hinf linear
t = 0:0.001:5;
u = zeros(size(t));
x0 = [0.1, 0, 0, 0];
Cz = [1 0 0 0; 0 1 0 0];
Dz = [0;0;1];
sys = ss(A + B*K, [0;0;0;0], [Cz;K], [0;0;0]);
[y,t,x] = lsim(sys, u, t, x0);
plot(t,y(:,1),'r')
hold on;
plot(t,y(:,2),'b')
hold on;
plot(t,y(:,3),'g')
axis([0 5 -20 20]);

%% Dados
%Dados utilizados na simulação
  I1 = 0.083;
  I2 = 0.33;
  m1 = 1;
  m2 = 1;
  l1 = 1;
  lc1 = 0.5;
  lc2 = 1;
  g = 9.81;
  
%% Espaço de estado
A = [0, 0, 1, 0; 0, 0, 0, 1; 12.6292, -12.6926, 0, 0; -14.7488, 29.6119, 0, 0];
B = [0; 0; -3.0147; 6.0332];
C = [ 1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 0, 0];
D = [0; 0; 1];
E = [0 0; 0 0; 1/I1 0; 0 1/I2];

%% Simulink
% ddq = H\[Bu - Cdq - G] = Mu + Ndq + O
%ddq1 = M11*u + N11*q1 + N12*q2 + O11
%ddq2 = M21*u + N21*q1 + N22*q2 + O21
syms q_1 q_2 q_1d q_2d
H = [m1*lc1^2 + m2*lc2^2 + I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q_2), m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2);
    m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2), m2*lc2^2 + I2];
B = [0;1];
C = [-2*m2*l1*lc2*sin(q_2)*q_2d, -m2*l1*lc2*sin(q_2)*q_2d;
    m2*l1*lc2*sin(q_2)*q_1d, 0];  
G = [(m1*lc1 + m2*l1)*g*sin(q_1) + m2*g*lc2*sin(q_1+q_2);
    m2*g*lc2*sin(q_1 + q_2)];
Hinv = inv(H);
M = Hinv*B;
N = -Hinv*C;
O = -Hinv*G;
M11 = M(1,1);
M21 = M(2,1);
N11 = N(1,1);
N12 = N(1,2);
N21 = N(2,1);
N22 = N(2,2);
O11 = O(1,1);
O21 = O(2,1);

plot(t,q1,'y');
hold on;
plot(t,q2,'g');
hold on;
plot(t,dq1,'b');
hold on;
plot(t,dq2,'r');