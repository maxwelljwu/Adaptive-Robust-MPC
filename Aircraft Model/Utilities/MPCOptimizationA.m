function u0 = MPCOptimizationA(pathName,x0,F_z,FHat,constraintParams,xRef)
%Runs the MPC optimization to find the optimal first step control

%x0 is the initial condition of the states
%thetaHat is the current estimate of the uncertainty parameters
%z_theta is the radius of the uncertainty set
%FHat is the estimate of the discrete time integration function. Inputs are current state and
    %control value. Output is next state
%ts is the sample period
%m is the mass

addpath(pathName);
import casadi.*;

N = 4; %Number of control actions in the time horizon

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

roll_lb = constraintParams.roll_lb;
pitch_lb = constraintParams.pitch_lb;
yaw_lb = constraintParams.yaw_lb;
p_lb = constraintParams.p_lb;
q_lb = constraintParams.q_lb;
r_lb = constraintParams.r_lb;
L_lb = constraintParams.Llb;
M_lb = constraintParams.Mlb;
N_lb = constraintParams.Nlb;


%Define optimal control problem
opti = casadi.Opti();

x = opti.variable(6,N+1); % Decision variables for state trajectory and lipschitz variable
z_x = opti.variable(1,N+1); %Worst case difference between predicted state and actual state
u = opti.variable(3,N); %Decision variable for input sequence
p = opti.parameter(6,1);  % Parameter (not optimized over)

opti.minimize((x(1,:)-xRef(1))*(x(1,:)-xRef(1))'+(x(2,:)-xRef(2))*(x(2,:)-xRef(2))' + ...
              (x(3,:)-xRef(3))*(x(3,:)-xRef(3))'+(x(4,:)-xRef(4))*(x(4,:)-xRef(4))' + ...
              (x(5,:)-xRef(5))*(x(5,:)-xRef(5))'+(x(6,:)-xRef(6))*(x(6,:)-xRef(6))');
          


for k=1:N
  opti.subject_to(x(:,k+1)==FHat(x(:,k),u(:,k)));
  opti.subject_to(z_x(k+1)==F_z(z_x(k)));
end
opti.subject_to(x(:,1)==p); %Set initial state condition
opti.subject_to(z_x(1)==0); %Set initial z_x condition
opti.subject_to(L_lb<=u(1, :)<=L_ub);
opti.subject_to(M_lb<=u(2, :)<=M_ub);
opti.subject_to(N_lb<=u(3, :)<=N_ub);
opti.subject_to(roll_lb<=x(1,:)-z_x);
opti.subject_to(x(1,:)+z_x<=roll_ub);
opti.subject_to(pitch_lb<=x(2,:)-z_x);
opti.subject_to(x(2,:)+z_x<=pitch_ub);
opti.subject_to(yaw_lb<=x(3,:)-z_x);
opti.subject_to(x(3,:)+z_x<=yaw_ub);
opti.subject_to(p_lb<=x(4,:)-z_x);
opti.subject_to(x(4,:)+z_x<=p_ub);
opti.subject_to(q_lb<=x(5,:)-z_x);
opti.subject_to(x(5,:)+z_x<=q_ub);
opti.subject_to(r_lb<=x(6,:)-z_x);
opti.subject_to(x(6,:)+z_x<=r_ub);

% Choose a concrete solver
opti.solver('sqpmethod',struct('qpsol','qrqp'));

% And choose a concrete value for p (in this case, p is the initial
% condition)
opti.set_value(p,x0);
sol = opti.solve();

u0 = sol.value(u(:,1));