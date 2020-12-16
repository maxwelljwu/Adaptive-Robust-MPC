%Location of the Casadi folder
pathName = 'C:\Users\Maxwell\Documents\MATLAB\Packages\casadi-windows-matlabR2016a-v3.5.5';

nt = 100; %Number of timesteps in simulation
x0 = [pi/6; -pi/6; pi/8; 0.1; -0.1; -0.1];
xRef = [pi/3; pi/3; 0; 0; 0; 0];

robustControl = false; %Boolean set to false for non-robust control and true for robust control
adaptiveControl = false; %Boolean set to false for non-adaptive control, and true for adaptive control

thetaHat0 = [0; 0; 0]; %Initial estimate of the uncertainty parameter
z_theta0 = 0.5; %Initial value of the radius of the uncertainty region
ts = 0.1; %Sample period

%Define model parameters
Ixx = 1;
Iyy = 4;
Izz = 6;
theta = [0.2; -0.1; -0.4]; %True value of uncertainty parameter


%% User Defined Parameters for adaptation algorithm
% Adaptive gain
gamma = 5*eye(3);

% Design Constant
K = 0.1*eye(6);

%% Constraint parameters
constraintParams.roll_ub = pi/4; %Upper bound on roll
constraintParams.pitch_ub = pi/4; %Upper bound on pitch
constraintParams.yaw_ub = pi/4; %Upper bound on yaw
constraintParams.p_ub = 0.5; %Upper bound on roll rate
constraintParams.q_ub = 0.5; %Upper bound on pitch rate
constraintParams.r_ub = 0.5; %Upper bound on yaw rate
constraintParams.Lub = 10; %Upper bound on input L (moment about x-axis)
constraintParams.Mub = 10; %Upper bound on input M (moment about y-axis)
constraintParams.Nub = 10; %Upper bound on input N (moment about z-axis)

constraintParams.roll_lb = -pi/4; %Lower bound on roll
constraintParams.pitch_lb = -pi/4; %Lower bound on pitch
constraintParams.yaw_lb = -pi/4; %Lower bound on yaw
constraintParams.p_lb = -0.5; %Lower bound on roll rate
constraintParams.q_lb = -0.5; %Lower bound on pitch rate
constraintParams.r_lb = -0.5; %Lower bound on yaw rate
constraintParams.Llb = -10; %Lower bound on input L (moment about x-axis)
constraintParams.Mlb = -10; %Lower bound on input M (moment about y-axis)
constraintParams.Nlb = -10; %Lower bound on input N (moment about z-axis)

if ~robustControl %If robust control is not used, turn off protections against uncertainty
    z_theta0 = 0; %Setting uncertainty radius to zero
end

if ~adaptiveControl %If adaptive control is not used, do not modify thetaHat
    gamma = zeros(size(gamma,1),size(gamma,2));
end

% Initial Conditions
xHat0 = x0;

%% Derived Initial Conditions for adaptation algorithm
eta0 = x0 - xHat0;
omega0 = zeros(length(x0), length(theta));
V_e_eta_t0 = 0.5*z_theta0^2;
Q0 = zeros(length(theta), length(theta));