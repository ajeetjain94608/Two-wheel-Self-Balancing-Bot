1;
Function_File;
pkg load symbolic      # Load the octave symbolic library
 alpha = sym('alpha');
  theta = sym('theta');
  theta_dot = sym('theta_dot');
  alpha_dot = sym('alpha_dot');
  
  u = sym('u');
   Mp = 0.5 ;                  # mass of the pendulum (Kg)
  l = 0.15 ;                  # length from pendulum's center of mass to pendulum's base/pivot (meter)
  g = 9.81 ;                  # Accelertion due to gravity (kgm/s^2)
  Ma = 0.25 ;                 # mass of the arm (kg)
 
  r_a = 0.01;                 # radius of arm cylinder (meter)
  r_p = 0.01;                 # radius of pendulum cylinder (meter)
 
  Rp = 0.2 ;                  # length from pendulum's base to arm's pivot point (meter)
  Ra = 0.1 ;                   # length from arm's center of mass to arm's pivot point (meter)
  
  I_arm = (1/3) * Ma *Rp^2;                   # Moment of inertia of the arm in yaw angle i.e. alpha (kgm^2)
  I_pendulum_theta = 0;        # Moment of inertia of the pendulum in tilt angle i.e. theta (kgm^2)
  I_pendulum_alpha = 0;        # Moment of inertia of the pendulum in yaw angle (kgm^2)
  

  
  cos_theta = cos(theta);
  sin_theta = sin(theta);

  cos_alpha = cos(alpha);
  sin_alpha = sin(alpha);
  k1=2*Mp*l*Rp;
  k2=2*Mp*l;
  k3=Mp*Rp*Rp;
  k4=Mp*l*l;
  k5=2*Mp*g*l;
  k6=I_arm+ k3;
  I_pen=(1/3) * Mp * 4*l^2;
  k7= I_pen + 4*k4;
  T=u;
  alpha_dot2=(T + k1*theta_dot*theta_dot)/k6;
  theta_dot2=(alpha_dot*theta_dot*sin_theta - 2*k1*alpha_dot*theta_dot*sin_theta +  k5*sin_theta ) / k7 ;
  
