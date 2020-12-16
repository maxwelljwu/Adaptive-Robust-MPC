function [F,Lf,Lg] = defineTrueDynamicsA(pathName,Ixx,Iyy,Izz,ts,theta,constraintParams)
addpath(pathName);
import casadi.*

%Constraint parameters
roll_ub = constraintParams.roll_ub;
pitch_ub = constraintParams.pitch_ub;
yaw_ub = constraintParams.yaw_ub;
p_ub = constraintParams.p_ub;
q_ub = constraintParams.q_ub;
r_ub = constraintParams.r_ub;
L_ub = constraintParams.Lub;
M_ub = constraintParams.Mub;
N_ub = constraintParams.Nub;


%Declare the variables. 6 states, 3 uncertainty parameter, 3 input
x = SX.sym('x',6); %The states and worst case deviationi between predicted and actual states
u = SX.sym('u',3); %The input

%% Define dynamics
%System dynamic equations
%f = [x(2);                              %Dynamics of x1
%    -k/m*x(1)-b/m*x(2)+1/m*u+1/m*theta];%Dynamics of x2

f = [x(4); 
    x(5); 
    x(6);
    (u(1)+theta(1))/Ixx - (Izz-Iyy)/Ixx*x(5)*x(6); %Dynamics of p
    (u(2)+theta(2))/Iyy - (Ixx-Izz)/Iyy*x(4)*x(6); %Dynamics of q
    (u(3)+theta(3))/Izz - (Iyy-Ixx)/Izz*x(4)*x(5)];%Dynamics of r

%Lf = 1.01/(x1ub^2+x2ub^2+uub^2)*(x2ub^2+(x1ub+x2ub+uub)^2); %Lipschitz constant on f function
Lf = 1;%1.01/(roll_ub^2+pitch_ub^2+yaw_ub^2+p_ub^2+q_ub^2+r_ub^2+L_ub^2+M_ub^2+N_ub^2); % need to improve this
Lg = 0; %Lipschitz constant on g function

%Create an integrator function
dae = struct('x',x,'p',u,'ode',f);

op = struct('tf',ts,'simplify',true,'number_of_finite_elements',6); % 4?
intg = integrator('intg','rk',dae,op);
res = intg('x0',x,'p',u);
x_next = res.xf;
F = Function('F',{x,u},{x_next},{'x','u'},{'x_next'});