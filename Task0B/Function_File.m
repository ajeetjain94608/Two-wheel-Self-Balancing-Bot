1;
## Function : find_equilibrium_points()
## ----------------------------------------------------
## Input:   x1_dot, x2_dot (Both are symbolic functions defined in terms of x1 
##          and x2)
## Output:  eqbm_points ( Solution in form of a matrix in which each entry is a 
##          solution)
## Purpose: This function takes the symbolic functions x1_dot and x2_dot and 
##          converts them into equations where x1_dot = 0  and x2_dot = 0. Then 
##          it proceeds to solve the equation (by calling the solve() function
##          in octave) and returns all possible values of x1 and x2 in form of a 
##          matrix.
function [x_1,x_2] = find_equilibrium_points(alpha_dot2,theta_dot2)
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
  theta_dot==0;
  alpha_dot==0;

  
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
  
alpha_dot2==0;
theta_dot2==0;
  [x_1,x_2]=solve([alpha_dot2,theta_dot2],[alpha,theta]);
    x_1= complex(x_1);
    x_2 = complex(x_2);
 
 
endfunction

[x_1,x_2] = find_equilibrium_points(alpha_dot2, theta_dot2)

