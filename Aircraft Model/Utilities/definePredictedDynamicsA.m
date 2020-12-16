function FHat = definePredictedDynamicsA(pathName,Ixx,Iyy,Izz,ts,thetaHat)
%Generates an estimate of the discrete time integration function based on 
%Estimated parameter, thetaHat. Inputs are current state and
    %control value. Output is the predicted next state

addpath(pathName);
import casadi.*
%Declare the variables. 6 states, 3 uncertainty parameter, 3 input, 1
%lipschitz variable
x = SX.sym('x',6); %The states and worst case deviation between predicted and actual states
u = SX.sym('u',3); %The input

%% Define dynamics
%System dynamic equations
%fHat = [x(2);                              %Dynamics of x1
%    -k/m*x(1)-b/m*x(2)+1/m*u+1/m*thetaHat];%Dynamics of x2

fHat = [x(4);
        x(5);
        x(6);
        (u(1)+thetaHat(1))/Ixx - (Izz-Iyy)/Ixx*x(5)*x(6); %Dynamics of p
        (u(2)+thetaHat(2))/Iyy - (Ixx-Izz)/Iyy*x(4)*x(6); %Dynamics of q
        (u(3)+thetaHat(3))/Izz - (Iyy-Ixx)/Izz*x(4)*x(5)];%Dynamics of r
%Create an integrator function
dae = struct('x',x,'p',u,'ode',fHat);

op = struct('tf',ts,'simplify',true,'number_of_finite_elements',6); % 4?
intg = integrator('intg','rk',dae,op);
res = intg('x0',x,'p',u);
x_next = res.xf;
FHat = Function('F',{x,u},{x_next},{'x','u'},{'x_next'});
end