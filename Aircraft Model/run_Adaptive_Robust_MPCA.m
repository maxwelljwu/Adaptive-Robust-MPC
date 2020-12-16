clear

defineSystemParametersA;
[F,Lf,Lg] = defineTrueDynamicsA(pathName,Ixx,Iyy,Izz,ts,theta,constraintParams); %Generate the integrator and Lipschitz constants for the true dynamics

%Create the state and control sequences. Set the initial condition
x = zeros(6,nt+1);
x(:,1) = x0;
xHat = zeros(6,nt+1);
xHat(:,1) = xHat0;
u = zeros(3,nt);
z_theta = zeros(nt+1,1);
z_theta(1)=z_theta0;
thetaHat = zeros(3,nt+1);
thetaHat(:,1) = thetaHat0;
eta = zeros(6,nt+1);
eta(:,1) = eta0;
Q = zeros(size(Q0,1),size(Q0,2),nt+1);
Q(:,:,1) = Q0;
omega = zeros(size(x,1),size(theta,1),nt+1);
omega(:,:,1) = omega0;
V_e_eta = zeros(nt+1,1);
V_e_eta(1) = V_e_eta_t0;
e = zeros(size(x,1),nt+1);
e(:,1) = x0-xHat0;
alpha = zeros(nt,1);


%Iterating over each timestep
for i = 1:nt
    disp(['Timestep ' num2str(i) ' of ' num2str(nt)]);
    
    FHat = definePredictedDynamicsA(pathName,Ixx,Iyy,Izz,ts,thetaHat(:,i)); %Create the predictive dynamic model based on the current estimate of theta
    F_z = defineZxDynamicsA(pathName,Lf,Lg,thetaHat(:,i),z_theta(i),Ixx,Iyy,Izz,ts); %Create the dynamic model for the estimation error bound based on the current estimate of theta and z_theta
    u(:,i) = MPCOptimizationA(pathName,x(:,i),F_z,FHat,constraintParams,xRef)'; %Solve the MPC problem to identify the next control input
    %x(:,i+1) = full(F(x(:,i),u(:,i))); %Calculate the state at the next timestep after applying the MPC control input
    uSim = [0 u(:,i)'; ts u(:,i)']; %The input for simulink simulation.
    simout = sim('parameter_and_set_adaptation'); %Simulate system and adaptation signals in response to MPC input

    %Progress signals to next timestep
    x(:,i+1) = simout.x.Data(end,:)';
    xHat(:,i+1) = simout.xHat.Data(end,:)';
    eta(:,i+1) = simout.eta.Data(end,:)';
    Q(:,:,i+1) = simout.Q.Data(:,:,end);
    omega(:,:,i+1) = simout.omega.Data(:,:,end);
    V_e_eta(i+1) = simout.V_e_eta.Data(end);
    e(:,i+1) = simout.e.Data(end,:)';
    alpha(i) = simout.alpha.Data(end);
    
    %Update the values of z_theta and thetaHat according to algorithm 1
    [z_theta,thetaHat] = algorithm1(simout,z_theta,thetaHat,i);
end
%%
clear i
makePlots;
