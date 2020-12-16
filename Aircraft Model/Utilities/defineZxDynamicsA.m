function F_z = defineZxDynamicsA(pathName,Lf,Lg,thetaHat,z_theta,Ixx,Iyy,Izz,ts)
%Generates an the discrete time integration function used to define the constraint
%given by Eq. 27c. Inputs are:

%Lf: Lipschitz constant on f function
%Lg: Lipschitz constant on g function
%thetaHat: Estimate of uncertainty parameter
%z_theta: radius of uncertainty set
%m: Mass

addpath(pathName);
import casadi.*

PI = z_theta+norm(thetaHat); %Pi parameter used in Eq. 25

%Declare the lipschitz variable
z = SX.sym('z'); %The worst case deviation between predicted and actual states

% %Dynamic equation for worst case difference between predicted and actual
% %states
g = zeros(6,3);
g(4,1) = 1/Ixx;
g(5,2) = 1/Iyy;
g(6,3) = 1/Izz;
fz_x = (Lf+Lg*PI)*z+norm(g)*z_theta;
dae_z = struct('x',z,'ode',fz_x);
op_z = struct('tf',ts,'simplify',true,'number_of_finite_elements',4);
intg_z = integrator('intg','rk',dae_z,op_z);
res = intg_z('x0',z);
z_next = res.xf;
F_z = Function('F',{z},{z_next},{'z'},{'z_next'});