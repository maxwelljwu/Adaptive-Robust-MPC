close all

time = 0:ts:nt*ts;
xIsZero = x==0;
x(xIsZero) = nan;
%Plot state and input trajectories
figure
subplot(3,2,1)
hold on
plot([0 time(end)],[xRef(1) xRef(1)],'k','LineStyle','-.','LineWidth',2);
plot(time,x(1,:),'Color',[0.2 0.4 0.8],'LineWidth',2);
plot([0 time(end)],[constraintParams.roll_ub constraintParams.roll_ub],'Color',[0.8 0.1 0.2],'LineWidth',2);
%plot([0 time(end)],[constraintParams.x1lb constraintParams.x1lb],'Color',[0.8 0.1 0.2],'LineWidth',2);
legend('$\phi_{ref}$','$\phi$','Constraints','FontSize',12)
ylabel('$\phi (rad)$','FontSize',14)
ylim([min(x(1,:))-0.8 xRef(1)+0.2])
subplot(3,2,3)
hold on
plot([0 time(end)],[xRef(2) xRef(2)],'k','LineStyle','-.','LineWidth',2);
plot(time,x(2,:),'Color',[0.2 0.4 0.8],'LineWidth',2);
plot([0 time(end)],[constraintParams.pitch_ub constraintParams.pitch_ub],'Color',[0.8 0.2 0.2],'LineWidth',2);
%plot([0 time(end)],[constraintParams.x2lb constraintParams.x2lb],'Color',[0.8 0.2 0.2],'LineWidth',2);
legend('$\gamma_{ref}$','$\gamma$','Constraints','FontSize',12)
ylabel('$\gamma (rad)$','FontSize',14)
ylim([min(x(2,:))-0.5 xRef(2)+0.2])

subplot(3,2,5)
hold on
plot([0 time(end)],[xRef(3) xRef(3)],'k','LineStyle','-.','LineWidth',2);
plot(time,x(3,:),'Color',[0.2 0.4 0.8],'LineWidth',2);
plot([0 time(end)],[constraintParams.yaw_ub constraintParams.yaw_ub],'Color',[0.8 0.2 0.2],'LineWidth',2);
legend('$\psi_{ref}$','$\psi$','Constraints','FontSize',12)
ylabel('$\psi (rad)$','FontSize',14)
ylim([xRef(3)-0.2 constraintParams.yaw_ub+0.2])
xlabel('Time (s)','FontSize',14)


subplot(3,2,2)
hold on
plot([0 time(end)],[xRef(4) xRef(4)],'k','LineStyle','-.','LineWidth',2);
plot(time,x(4,:),'Color',[0.2 0.4 0.8],'LineWidth',2);
plot([0 time(end)],[constraintParams.p_ub constraintParams.p_ub],'Color',[0.8 0.2 0.2],'LineWidth',2);
legend('$p_{ref}$','$p$','Constraints','FontSize',12)
ylim([xRef(4)-0.1 constraintParams.p_ub+0.1])
ylabel('$p (rad/s)$','FontSize',14)

subplot(3,2,4)
hold on
plot([0 time(end)],[xRef(5) xRef(5)],'k','LineStyle','-.','LineWidth',2);
plot(time,x(5,:),'Color',[0.2 0.4 0.8],'LineWidth',2);
plot([0 time(end)],[constraintParams.q_ub constraintParams.q_ub],'Color',[0.8 0.2 0.2],'LineWidth',2);
legend('$q_{ref}$','$q$','Constraints','FontSize',12)
ylabel('$q (rad/s)$','FontSize',14)
ylim([xRef(5)-0.2 constraintParams.q_ub+0.1])

subplot(3,2,6)
hold on
plot([0 time(end)],[xRef(6) xRef(6)],'k','LineStyle','-.','LineWidth',2);
plot(time,x(6,:),'Color',[0.2 0.4 0.8],'LineWidth',2);
plot([0 time(end)],[constraintParams.r_ub constraintParams.r_ub],'Color',[0.8 0.2 0.2],'LineWidth',2);
legend('$r_{ref}$','$r$','Constraints','FontSize',12)
xlabel('Time (s)','FontSize',14)
ylabel('$r (rad/s)$','FontSize',14)
ylim([xRef(6)-0.3 constraintParams.r_ub+0.1])

sgtitle('Traditional MPC')

figure
subplot(3,1,1)
stairs(time, [u(1,:) nan],'Color',[0.1 0.5 0.2],'LineWidth',2);
hold on
plot([0 time(end)],[constraintParams.Lub constraintParams.Lub],'Color',[0.8 0.2 0.4],'LineWidth',2);
%plot([0 time(end)],[constraintParams.ulb constraintParams.ulb],'Color',[0.8 0.2 0.4],'LineWidth',2);
legend('$\mathcal{L}$','Constraints','FontSize',12)
ylabel('$\mathcal{L} (Nm)$','FontSize',14)
ylim([min(u(1,:))-1 constraintParams.Lub+1])

subplot(3,1,2)
stairs(time, [u(2,:) nan],'Color',[0.1 0.5 0.2],'LineWidth',2);
hold on
plot([0 time(end)],[constraintParams.Mub constraintParams.Mub],'Color',[0.8 0.2 0.4],'LineWidth',2);
%plot([0 time(end)],[constraintParams.ulb constraintParams.ulb],'Color',[0.8 0.2 0.4],'LineWidth',2);
legend('$\mathcal{M}$','Constraints','FontSize',12)
ylabel('$\mathcal{M} (Nm)$','FontSize',14)
ylim([min(u(2,:))-1 constraintParams.Mub+1])

subplot(3,1,3)
stairs(time, [u(3,:) nan],'Color',[0.1 0.5 0.2],'LineWidth',2);
hold on
plot([0 time(end)],[constraintParams.Nub constraintParams.Nub],'Color',[0.8 0.2 0.4],'LineWidth',2);
%plot([0 time(end)],[constraintParams.ulb constraintParams.ulb],'Color',[0.8 0.2 0.4],'LineWidth',2);
legend('$\mathcal{N}$','Constraints','FontSize',12)
ylabel('$\mathcal{N} (Nm)$','FontSize',14)
ylim([min(u(3,:))-1 constraintParams.Nub+1])
xlabel('Time (s)','FontSize',14)
sgtitle('u')

figure
subplot(2,1,1)
hold on
% hold on
% plot([0 time(end)],[theta theta],'Color','k','LineWidth',2,'LineStyle','-.')
plot(time,sqrt((theta(1,:)-thetaHat(1,:)).^2),'Color',[0.2 0.4 0.8],'LineWidth',2);
plot(time,sqrt((theta(2,:)-thetaHat(2,:)).^2),'Color',[0.8 0.2 0.4],'LineWidth',2);
plot(time,sqrt((theta(3,:)-thetaHat(3,:)).^2),'Color',[0.4 0.8 0.2],'LineWidth',2);
% xlabel('Time (s)','FontSize',16);
ylabel('$||\theta-\hat{\theta}||$','FontSize',16);
legend('$|\theta_\mathcal{L}-\hat{\theta}_{\mathcal{L}}|$','$|\theta_\mathcal{M}-\hat{\theta}_{\mathcal{M}}|$','$|\theta_\mathcal{N}-\hat{\theta}_{\mathcal{N}}|$','FontSize',14);

subplot(2,1,2)
stairs(time,z_theta,'Color',[0.2 0.4 0.8],'LineWidth',2);
xlabel('Time (s)','FontSize',16);
ylabel('$z_\theta$','FontSize',16);