model_parameters

for i = 1:length(force_scale)
    
    %p0 = [pos_offset(i)+0 0 0 0 0 0]';
    %pf = [pos_offset(i)+0.2 0 1.2 0 0 0]';
    
    %force_v(floor(Mld.nControlPoints/2),1) = 0*force_scale(i); % floor(length(u)/2),1)
    %force_v(floor(Mld.nControlPoints/2),2) = 0*force_scale(i);
    force_v(7,3) = 0;%*force_scale(i);
    
    %force_v(:,2) = 1*ones(Mld.nControlPoints,1);
    %force_v(:,3) = -0.01*ones(Mld.nControlPoints,1);
    
    torque (floor(Mld.nControlPoints/2),1) = 0;
    
    % this could be used for designing a force/torque profile in time
    Fext = cell (Mld.nControlPoints,4);
    for f1 =1: Mld.nControlPoints
        for f2=1:3
            Fext{f1,f2} = [force_v(f1,f2) force_v(f1,f2)];
        end
        Fext{f1,4} = [torque(f1) torque(f1)];
    end
    % Equilibrium
    %[X_,fval, exitflag] = solveEquilibrium(Mld,[Tsim(i) Tsim(i+1)],Yinit,Fext,Econst,options); % Call solver
        
    % simulation of system
    % Y [size] = [lenght(simTime),8*#controlpoints]
    % the system can be simulated around the EP X_ calculated in the
    % previous step
    [T, Y] = simulateSys(Mld,[Tsim(i) Tsim(i+1)],Yinit,Fext,Econst,OPTIONS);
    Yinit = vec2mat(Y(end,:),Mld.nControlPoints)';
    % Yinit = Y;
    
    % T: is the simulation time from T(k) to T(k+1)
    % Y: values of control points in each time step
    
    % control points in time
    Q   = Y(:,1:4*Mld.nControlPoints);            
    %  control points first derivative in time  
    dQ  = Y(:,4*Mld.nControlPoints+1:end);
    
    % initialize the coordinate matrices/time
    x_t = zeros (length(T),Mld.nSplineSamples);
    y_t = zeros (length(T),Mld.nSplineSamples);
    z_t = zeros (length(T),Mld.nSplineSamples);
    theta_t = zeros (length(T),Mld.nSplineSamples);
    
    % re-calculate the position and rotation fields for each time step
    for t=length(T)
        qi       = vec2mat(Q(t,:),Mld.nControlPoints);   
    
        r = Mld.b'*qi(1:3,:)';
        Theta = Mld.b'*qi(4,:)';

        x_t(t,:)     = r(:,1)';
        y_t(t,:)     = r(:,2)';
        z_t(t,:)     = r(:,3)';  
        theta_t(t,:) = Theta';
            
        x_v = x_t(t,:);
        y_v = y_t(t,:);
        z_v = z_t(t,:);  
        theta_v = theta_t(t,:);
        
        figure(2)
        h = plot3(x_v,y_v,z_v,'linewidth',2);
        P=qi(1:3,:);
        plot3(P(1,:),P(2,:),P(3,:),'m-.')
        plot3(P(1,:),P(2,:),P(3,:),'mo')
        %view(144,18);
        %xlim([p0(3) pf(3)])
        xlabel('x')
        ylabel('y')
        zlabel('z')
        %ylim([-0.1 0.1])    

%         scale = .00005;
%         % plot force vectors
%         zlim([floor_level pf(1)])
%         for j = 1:length(force_v)
%            if force_v(j,:) ~= 0
%                 v = quiver3(z_v(j),y_v(j),x_v(j),scale*force_v(j,3),scale*force_v(j,2),scale*force_v(j,1),'r','linewidth',2);
%            end
%         end
        drawnow;
        %F(i) = getframe(gcf);
        %delete(h);
    end
end

% save 

