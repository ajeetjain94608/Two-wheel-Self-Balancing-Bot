1;

pkg load control;
pkg load symbolic;

##**************************************************************************
##*                OCTAVE PROGRAMMING (e-Yantra)
##*                ====================================
##*  This software is intended to teach Octave Programming and Mathematical
##*  Modeling concepts
##*  Theme: Lunar Scout
##*  Filename: Task_1A.m
##*  Version: 1.0.0  
##*  Date: 18/09/2023
##*
##*  Team ID :
##*  Team Leader Name:
##*  Team Member Name
##*
##*  
##*  Author: e-Yantra Project, Department of Computer Science
##*  and Engineering, Indian Institute of Technology Bombay.
##*  
##*  Software released under Creative Commons CC BY-NC-SA
##*
##*  For legal information refer to:
##*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
##*     
##*
##*  This software is made available on an �AS IS WHERE IS BASIS�. 
##*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
##*  any and all claim(s) that emanate from the use of the Software or 
##*  breach of the terms of this agreement.
##*  
##*  e-Yantra - An MHRD project under National Mission on Education using 
##*  ICT(NMEICT)
##*
##**************************************************************************

## Function : Jacobian_A_B()
## ----------------------------------------------------
## Input:   Mp                - mass of the pendulum
##          l                 - Length of Pendulum
##          g                 - Acceleration due to gravity
##          Ma                - mass of the arm
##          Rp                - length of pendulum base from the pivot point
##          Ra                 - length from arm's center of mass to arm's pivot point
##          I_arm             - Moment of inertia of the arm in yaw angle
##          I_pendulum_theta  - Moment of inertia of the pendulum in tilt angle
##          I_pendulum_alpha  - Moment of inertia of the pendulum in yaw angle
##
## Output:  A - A matrix of system (State or System Matrix )
##          B - B matrix of system (Input Matrix)
##          
## Purpose: Use jacobian function to find A and B matrices(State Space Model) in this function.

function [A,B] = Jacobian_A_B(Mp,l,g,Ma,Rp,Ra,I_arm,I_pendulum_theta,I_pendulum_alpha)

  alpha = sym('alpha');
  theta = sym('theta');
  theta_dot = sym('theta_dot');
  alpha_dot = sym('alpha_dot');
  u = sym('u');
  
  cos_theta = cos(theta);
  sin_theta = sin(theta);

  cos_alpha = cos(alpha);
  sin_alpha = sin(alpha);
  k1=2*Mp*l*Rp;
  k2=2*Mp*l;
  k3=Mp*Rp^2;
  k4=Mp*l^2;
  k5=2*g*Mp*l;
  k6= I_arm + k3;
  k7 = I_pendulum_theta+4*k4;
  T = u;
  alpha_dot2 = ((-T*k1*cos_theta) - (k1^2*theta_dot^2*sin_theta*cos_theta) + (2*k1*k6*alpha_dot*theta_dot*sin_theta) - (k5*k6*sin_theta) - (alpha_dot*theta_dot*k6*sin_theta))/ ((-k1^2*cos_theta^2) - (k6*k7));
  theta_dot2 = ((cos_theta*sin_theta)*((k1*alpha_dot*theta_dot) - (2*k1^2*alpha_dot*theta_dot) + (k1*k5)) - (k7*T) - (k1*k7*theta_dot^2*sin_theta))/ ((k1^2*cos_theta^2) - (k7*k6));
  x=[alpha_dot,alpha,theta_dot,theta];
  expressions = [alpha_dot2,alpha_dot,theta_dot2,theta_dot];
  jacobian_matrices = {};
   jacobian_matrices1 = {};
  jacobian_matrix = jacobian(expressions,x);
  
  jacobian_matrices = subs(jacobian_matrix, {alpha_dot,alpha,theta_dot,theta}, {0,0,0,pi});
  jacobian_matrices=  double(jacobian_matrices);
  A = jacobian_matrices;
  jacobian_matrix1 = jacobian(expressions,T);
  jacobian_matrices1 = subs(jacobian_matrix1, {alpha_dot,alpha,theta_dot,theta}, {0,0,0,pi});
   jacobian_matrices1=  double(jacobian_matrices1);
    B = jacobian_matrices1;
   
  ########## ADD YOUR CODE HERE ################
  #{
  Steps : 
    1. Define equations of motion (4-states so 4 equations). It is suggested to use Lagrangian method. You can try Newtonian methods too.
    2. Partial Differentiation of equations of motion to find the Jacobian matrices
    3. Linearization by substituting equillibrium point condition in Jacobians
  
  ### NOTE ### : Sequence of states should not be altered for evaluation
  
  SEQUENCE OF STATES : [alpha_dot; alpha; theta_dot; theta]
        Example:
                A = [x x x x;   # corresponds to ...alpha_dot
                     x x x x;   # ...alpha
                     x x x x;   # ...theta_dot
                     x x x x]   # ...theta
                B = [x;   # ...alpha_dot
                     x;   # ...alpha
                     x;   # ...theta_dot
                     x]   # ...theta  
  #}
  ##############################################  
  #A = ; # A should be (double) datatype 
 # B = ; # B should be (double) datatype
  
endfunction

## Function : lqr_Rotary_Inverted_Pendulum()
## ----------------------------------------------------
## Input:   A - A matrix of system (State or System Matrix )
##          B - B matrix of system (Input Matrix)
##
## Output:  K - LQR Gain Matrix
##          
## Purpose: This function is used for finding optimal gains for the system using
##          the Linear Quadratic Regulator technique           

function K = lqr_Rotary_Inverted_Pendulum(A,B)
  C    =  [1 0 0 0;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];           ## Initialise C matrix
  D     = [0;0;0;0];          ## Initialise D matrix
   
 Q = [5 0 0 0;    % Cost associated with alpha_dot
     0 75 0 0;      % Cost associated with alpha (emphasizing alpha)
     0 0 7 0;   % Cost associated with theta_dot
     0 0 0 200];    % Cost associated with theta

  R     = 1;                  ## Initialise R 
  sys   = ss(A,B,C,D);        ## State Space Model
  K     = lqr(sys,Q,R);       ## Calculate K matrix from A,B,Q,R matrices using lqr()
  
endfunction

## Function : Rotary_Inverted_Pendulum_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions.
##          (1) Tilt angle is represented as theta
##          (2) Yaw angle is represented as alpha
        
function Rotary_Inverted_Pendulum_main()
  
  Mp = 0.2 ;                  # mass of the pendulum (Kg)
  l = 0.075 ;                  # length from pendulum's center of mass to pendulum's base/pivot (meter)
  g = 9.81 ;                  # Accelertion due to gravity (kgm/s^2)
  Ma = 0.587 ;                 # mass of the arm (kg)
 
  r_a = 0.02;                 # radius of arm cylinder (meter)
  r_p = 0.02;                 # radius of pendulum cylinder (meter)
 
  Rp = 0.15 ;                  # length from pendulum's base to arm's pivot point (meter)
  Ra = 0.075 ;                   # length from arm's center of mass to arm's pivot point (meter)
  
  I_arm = (1/4)*Ma*r_a^2 + (1/3)*Ma*Rp^2 ;                   # Moment of inertia of the arm in yaw angle i.e. alpha (kgm^2)
  I_pendulum_theta = (1/4)*Mp*r_p^2 + (1/3)*Mp*l^2*4 ;        # Moment of inertia of the pendulum in tilt angle i.e. theta (kgm^2)
  I_pendulum_alpha = 0;        # Moment of inertia of the pendulum in yaw angle (kgm^2)
  
  [A,B] = Jacobian_A_B(Mp,l,g,Ma,Rp,Ra,I_arm,I_pendulum_theta,I_pendulum_alpha) ## find A , B matrix using  Jacobian_A_B() function
  K = lqr_Rotary_Inverted_Pendulum(A,B)  ## find the gains using lqr_Rotary_Inverted_Pendulum() function
  
endfunction