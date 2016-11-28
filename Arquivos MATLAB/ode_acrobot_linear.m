function dy = ode_acrobot_linear(t,y,K)

  I1 = 0.083;
  I2 = 0.33;
  m1 = 1;
  m2 = 1;
  l1 = 1;
  lc1 = 0.5;
  lc2 = 1;
  g = 9.81;

  q_1e = pi;
  q_2e = 0;
  q_1de = 0;
  q_2de = 0;
  
  q_1 = y(1);
  q_2 = y(2);
  q_1d = y(3);
  q_2d = y(4);
  q = [q_1;q_2;q_1d;q_2d];
  
  u = -K*q;
  %u=0;
  
  He = [ m1*lc1^2 + m2*lc2^2 + I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q_2e), m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2e);
        m2*lc2^2 + I2 + m2*l1*lc2*cos(q_2e),                  m2*lc2^2 + I2                       ];
    
  Ce = [ -2*m2*l1*lc2*sin(q_2e)*q_2de, -m2*l1*lc2*sin(q_2e)*q_2de;
        m2*l1*lc2*sin(q_2e)*q_1de,    0                         ];
    
  %G = [ (m1*lc1 + m2*l1)*g*sin(q_1) + m2*g*lc2*sin(q_1+q_2);
  %      m2*g*lc2*sin(q_1 + q_2)                              ];
    
  Be = [0;1];
  
  dGdq = [ -g*(m1*lc1 + m2*l1 + m2*lc2), -m2*g*lc2;
           -m2*g*lc2,                -m2*g*lc2 ];
  
  
  A = [ 0 0 1 0; 0 0 0 1; -He\dGdq, -He\Ce ];
  B = [0;0;He\Be];
  Ee = [1 0;0 1];
  E = [0 0;0 0;He\Ee];
  
  %dy = A*(y - [pi;0;0;0]) + B*u;
  dy = A*y + B*u;