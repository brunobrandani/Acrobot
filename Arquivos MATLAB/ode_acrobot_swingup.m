function dy = ode_acrobot(t,y,K,k1,k2,alfa)

  
  I1 = 4.3399e-3;
  I2 = 5.2285e-3;
  m1 = 1.9008;
  m2 = 0.7175;
  l1 = 0.2;
  lc1 = 1.8522e-1;
  lc2 = 6.2052e-2;
  g = 9.81;


  I1 = 0.083;
  I2 = 0.33;
  m1 = 1;
  m2 = 1;
  l1 = 1;
  lc1 = 0.5;
  lc2 = 1;
  g = 9.81;
  
  q_1 = y(1);
  q_2 = y(2);
  q_1d = y(3);
  q_2d = y(4);
  q = [q_1+pi;q_2;q_1d;q_2d];
  q_dot = [q_1d;q_2d];
  
  dy = zeros(4,1);
  dy(1) = q_1d;
  dy(2) = q_2d;
  
  %u = 0;
  
  %Hinf
  %u = -1.0e+03 *[3.1944    1.4330    1.3982    0.6862]*q;
  
  %LQR
  %u = -K*q;
  
  %swing up
  %alfa = 3;
  %Kp = 16;
  %Kd = 8;
  %q2D = alfa*atan(q_1d);
  %u = Kp*(q2D - q_2) - Kd*q_2d;
  
  % Saturação
  %u = max(min(u,20),-20);
  
   
  H = [ m1*lc1^2 + m2*lc2^2 + I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q_2), m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2);
        m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2),                  m2*lc2^2 + I2                       ];
    
  C = [ -2*m2*l1*lc2*sin(q_2)*q_2d, -m2*l1*lc2*sin(q_2)*q_2d;
        m2*l1*lc2*sin(q_2)*q_1d,    0                         ];
    
  G = [ (m1*lc1 + m2*l1)*g*sin(q_1) + m2*g*lc2*sin(q_1+q_2);
        m2*g*lc2*sin(q_1 + q_2)                              ];
    
  B = [0;1];
  
  % Calculo de energia
  Etopo = 2*((m1*lc1 + m2*l1 + m2*lc2)*g);
  Epotencial = -(m1*lc1 + m2*l1)*g*cos(q_1) - m2*g*lc2*cos(q_1+q_2) + ((m1*lc1 + m2*l1 + m2*lc2)*g);
  Ecinetica = 1/2*q_dot'*H*q_dot;
  Eatual = Epotencial + Ecinetica;
  Eerro = Etopo-Eatual; 
  
  % Troca para LQR
  LQR = y(5);
  
  if(abs(q_1) > 3.1 && LQR == 0)
  %if(abs(q_1) > 33.2 && LQR == 0)
      LQR = 1;
  end
  
  if(LQR ~= 0)
      dy(5) = 1;
      u = -K*q;
      
  % Swing up    
  else
      dy(5) = 0;
      q2d = alfa*atan(q_1d);
      u = k1*(q2d-q_2) - k2*q_2d;
      u = sat(u,20);
      
      %k1limit = 50000;
      %u = sat(k1*atan(q_1d)*Eerro - k2*q_2 - alfa*q_2d,50);
      %u = k1*atan(q_1d)*Eerro - k2*q_2 - alfa*q_2d;
  end
  
  %u = 0;
  % Retorno sistema
  dy(3:4) = H\(B*u - G - C*[q_1d;q_2d]);
end



function z = sat(x,y)
    z = max(min(x,y),-y);
end
  