function dy = ode_acrobot_mod_param(t,y,K)

  %% Parâmetros variáveis   
  
  var_vector = [0.0808    0.2554   40.9769   56.1507  130.0000   67.8724  137.7485]/1000;
  %ref:         0.0094    0.0284    4.4441    5.7771   13.0000    6.8489   13.3961 /100
  
  I1  = var_vector(1);
  I2  = var_vector(2);
  m1  = var_vector(3);
  m2  = var_vector(4);
  l1  = var_vector(5);
  lc1 = var_vector(6);
  lc2 = var_vector(7);
  g = 9.81;
  
  q_1 = y(1);
  q_2 = y(2);
  q_1d = y(3);
  q_2d = y(4);
  q = [q_1-pi;q_2;q_1d;q_2d];
  %q_dot = [q_1d;q_2d];
  
  dy = zeros(4,1);
  dy(1) = q_1d;
  dy(2) = q_2d;
  
  %u = 0;
  
  %Hinf
  %u = -1.0e+03 *[3.1944    1.4330    1.3982    0.6862]*q;
  
%   %LQR
   u = -K*q;
%   % Saturação
   u = max(min(u,1.2),-1.2);
%   
%   Kp = 10;
%   Kd = 1;
%   u = (Kp*(0.505*atan(q_1d) - q_2) - Kd*q_2d);
%   u = max(min(u,1.2),-1.2);
  
  H = [ m1*lc1^2 + m2*lc2^2 + I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q_2), m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2);
        m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2),                  m2*lc2^2 + I2                       ];
    
  C = [ -2*m2*l1*lc2*sin(q_2)*q_2d, -m2*l1*lc2*sin(q_2)*q_2d;
        m2*l1*lc2*sin(q_2)*q_1d,    0                         ];
    
  G = [ (m1*lc1 + m2*l1)*g*sin(q_1) + m2*g*lc2*sin(q_1+q_2);
        m2*g*lc2*sin(q_1 + q_2)                              ];
    
  B = [0;1];
  
%   % Controlador swing up 1
%   % Constantes de controle
%   Kv = 0;
%   Kp = 0;
%   Kd = 0;
%   Ke = 0;
%   % Energia
%   Etop = ((m1*lc1 + m2*l1 + m2*lc2)*g);
%   Epot = (m1*lc1 + m2*l1)*g*sin(q_1) + m2*g*lc2*sin(q_1+q_2);
%   Ecin = 1/2*q_dot'*H*q_dot;
%   Ecur = Epot + Ecin;
%   Eerr = Ecur - Etop;
%   delta = det(H);
%   d21 = H(2,1);
%   h1 = C(1);
%   h2 = 0;
%   phi1 = 0;
%   phi2 = 0;
%   d11 = 0;
%   u0 = (-(Kv*q_2d + Kp*q_2)*delta - Kd*(d21*(h1+phi1) - d11*(h2 + phi2)))/(Ke*Eerr*delta + Kd*d11);
  
  
  dy(3:4) = H\(B*u - G - C*[q_1d;q_2d]);