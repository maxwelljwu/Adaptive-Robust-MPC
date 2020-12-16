function [z_theta,thetaHat] = algorithm1(simout,z_theta,thetaHat,i)
    z_theta_ti = z_theta(i);
    thetaHat_ti = thetaHat(:,i);
    
    z_theta_current = simout.z_theta_current.Data(end);
    thetaHat_current = simout.theta_hat.Data(end,:)';
    
    if z_theta_current <= z_theta_ti - norm(thetaHat_current-thetaHat_ti)
        z_theta(i+1) = z_theta_current;
        thetaHat(:,i+1) = thetaHat_current;
    else
        z_theta(i+1) = z_theta_ti;
        thetaHat(:,i+1) = thetaHat_current;%thetaHat_ti; %Changing the thetaHat update algorithm from the paper.
    end      
end