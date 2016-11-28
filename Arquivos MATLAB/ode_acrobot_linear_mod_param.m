function dy = ode_acrobot_linear_mod_param(t,y)
  % Densidade do material do pendulo (alumínio) - kg/m3
  dalum = 2700;
  
  % Parâmetros de construção do pêndulo 1
  barra1_d1  = 0.011;
  barra1_d2  = 0.006;
  barra1_d3  = 0.011;
  barra1_d4  = 0.016;
  barra1_d5  = 0.006;
  barra1_l1  = 0.130;
  barra1_e1  = 0.011;
  barra1_e2  = 0.020;
  barra1_e3  = 0.011;
  barra1_e4  = 0.010;
  
  % Parâmetros de construção do pêndulo 2
  barra2_d6  = 0.016;
  barra2_d7  = 0.006;
  barra2_d8  = 0.008;
  barra2_d9  = 0.006;
  barra2_l2  = 0.180;
  barra2_c1  = 0.036;
  barra2_c2  = 0.015;
  barra2_c3  = 0.020;
  barra2_e5  = 0.007;
  barra2_e6  = 0.005;
  
  % Cálculo da massa de cada componente do pêndulo 1
  barra1_mI    = pi*barra1_d1^2*barra1_e1*dalum/4;
  barra1_mII   = pi*barra1_d2^2*barra1_e2*dalum/4;
  barra1_mIII  = pi*barra1_d3^2*(barra1_l1 - barra1_d1/2 - barra1_d4/2)*dalum/4;
  barra1_mIV   = pi*barra1_d4^2*barra1_e3*dalum/4;
  barra1_mV    = pi*barra1_d5^2*barra1_e4*dalum/4;
  barra1_m     = (barra1_mI + barra1_mII + barra1_mIII + barra1_mIV + barra1_mV);
  
  % Cálculo do centro de massa do pêndulo 1 (em relação ao eixo de rotação)
  barra1_cg = (barra1_mIII*(barra1_l1 + barra1_d1/2 - barra1_d4/2)/2 + barra1_l1*(barra1_mIV + barra1_mV))/barra1_m; 
  
  % Cálculo do momento de inércia de cada componente do pêndulo 1 (em
  % relação ao centro de massa do pêndulo 1)
  barra1_I_I   = (barra1_mI*barra1_d1^2)/8  + barra1_mI*(barra1_cg)^2;
  barra1_I_II  = (barra1_mII*barra1_d2^2)/8 + barra1_mII*(barra1_cg)^2;
  barra1_I_III = barra1_mIII*((barra1_d3^2)/16 + ((barra1_l1 - barra1_d1/2 - barra1_d4/2)^2)/12 + (barra1_cg - (barra1_l1 + barra1_d1/2 - barra1_d4/2)/2)^2);
  barra1_I_IV  = barra1_mIV*((barra1_d4^2)/8 + (barra1_cg - barra1_l1)^2);
  barra1_I_V   = barra1_mV*((barra1_d5^2)/8  + (barra1_cg - barra1_l1)^2);
  
  % Cálculo do momento de inércia do pêndulo 1 (em relação ao seu centro de
  % massa)
  barra1_I = (barra1_I_I + barra1_I_II + barra1_I_III + barra1_I_IV + barra1_I_V);
  
  % Cálculo do momento de inércia do pêndulo 1 dividido pela sua massa (em
  % relação ao seu centro de massa)
  barra1_I_invMass = barra1_I/barra1_m;
  
  % Barra 2
  
  % Cálculo da massa de cada componente do pêndulo 2
  barra2_mVI   = pi*(barra2_d6^2 - barra2_d9^2)*barra2_e5*dalum/4;
  barra2_mVII  = pi*barra2_d7^2*barra2_e6*dalum/4;
  barra2_mVIII = pi*barra2_d8^2*(barra2_l2 - barra2_d6/2)*dalum/4;
  barra2_mIX   = barra2_c1*barra2_c2*barra2_c3*dalum;
  barra2_m     = (barra2_mVI + barra2_mVII + barra2_mVIII + barra2_mIX);
  
  % Cálculo do centro de massa do pêndulo 1 (em relação ao eixo de rotação)
  barra2_cg    = (barra2_mVIII*(barra2_l2 + barra2_d6/2)/2 + barra2_mIX*(barra2_l2 + barra2_c3/2))/barra2_m;
  
  % Cálculo do momento de inércia de cada componente do pêndulo 1 (em
  % relação ao centro de massa do pêndulo 1)
  barra2_I_VI   = barra2_mVI*(barra2_d6^2 + barra2_d9^2)/8  + barra2_mVI*(barra2_cg)^2;
  barra2_I_VII  = barra2_mVII*(barra2_d7^2)/8 + barra2_mVII*(barra2_cg)^2;
  barra2_I_VIII = barra2_mVIII*((barra2_d8^2)/16 + ((barra2_l2 - barra2_d6/2)^2)/12 + (barra2_cg - (barra2_l2 + barra2_d6/2)/2)^2);
  barra2_I_IX   = barra2_mIX*((barra2_c1^2 + barra2_c3^2)/12 + (barra2_cg - (barra2_c3/2 + barra2_l2))^2);
  
  % Cálculo do momento de inércia do pêndulo 1 (em relação ao seu centro de
  % massa)
  barra2_I = (barra2_I_VI + barra2_I_VII + barra2_I_VIII + barra2_I_IX);

  % Cálculo do momento de inércia do pêndulo 1 dividido pela sua massa (em
  % relação ao seu centro de massa)
  barra2_I_invMass = barra2_I/barra2_m;
  
  % Calculos
  I1 = barra1_I;
  I2 = barra2_I;
  m1 = barra1_m;
  m2 = barra2_m;
  l1 = barra1_l1;
  lc1 = barra1_cg;
  lc2 = barra2_cg;
  g = 9.81; 
  
  %Dados para V-REP
  massa_1 = barra1_m;
  inercia_1 = barra1_I/barra1_m;
  cg_1 = barra1_cg - barra1_l1/2;
  
  massa_2 = barra2_m;
  inercia_2 = barra2_I/barra2_m;
  cg_2 = barra2_cg - barra2_l2/2;
  
  
 1000*[barra1_I barra2_I barra1_m barra2_m barra1_l1 barra1_cg barra2_cg]

  q_1e = pi;
  q_2e = 0;
  q_1de = 0;
  q_2de = 0;
  
  %q_1 = y(1);
  %q_2 = y(2);
  %q_1d = y(3);
  %q_2d = y(4);
  %q = [q_1;q_2;q_1d;q_2d];
  
  %u = -K*q;
  u=0;

  q_1e = pi;
  q_2e = 0;
  q_1de = 0;
  q_2de = 0;
  
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
  
  %%
  %dy = A*(y - [pi;0;0;0]) + B*u;
  dy = A*y + B*u;