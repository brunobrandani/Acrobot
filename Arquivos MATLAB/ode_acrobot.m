%Função para simular dinâmica do sistema Acrobot
function dy = ode_acrobot(t,y,K)

  %Parâmetros do sistema
  I1 = 0.083;
  I2 = 0.33;
  m1 = 1;
  m2 = 1;
  l1 = 1;
  lc1 = 0.5;
  lc2 = 1;
  g = 9.81;

  %Variáveis
  q_1 = y(1);
  q_2 = y(2);
  q_1d = y(3);
  q_2d = y(4);
  q = [q_1-pi;q_2;q_1d;q_2d];
  q_dot = [q_1d; q_2d];
  
  dy = zeros(4,1);
  dy(1) = q_1d;
  dy(2) = q_2d;
  
  % Controlador linear
  u = -K*q;
  % Saturação (se necessário)
  %u = max(min(u,20),-20);  
  
  %Matrizes para a forma de manipulador
  H = [ m1*lc1^2 + m2*lc2^2 + I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q_2), m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2);
        m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2),                  m2*lc2^2 + I2                       ];
    
  C = [ -2*m2*l1*lc2*sin(q_2)*q_2d, -m2*l1*lc2*sin(q_2)*q_2d;
        m2*l1*lc2*sin(q_2)*q_1d,    0                         ];
    
  G = [ (m1*lc1 + m2*l1)*g*sin(q_1) + m2*g*lc2*sin(q_1+q_2);
        m2*g*lc2*sin(q_1 + q_2)                              ];
    
  B = [0;1];  
  
  dy(3:4) = H\(B*u - G - C*[q_1d;q_2d]);