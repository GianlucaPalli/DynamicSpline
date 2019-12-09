function [T, Y] = simulateSys(Mld,Tm,y0,Fext,E,OPTIONS)

% simulation
tspan = [min(Tm),max(Tm)];
% y0 = y0(:);  
y1=y0(:,1:Mld.p+1);                   % control points
y2=y0(:,(Mld.p+1)+1:2*(Mld.p+1));     % derivative of control points
y0 = [y1(:);y2(:)];                   % conversion of matrix into a colum vec such that:
                                      % yo = [qi_x
                                      %       qi_y
                                      %       qi_z 
                                      %       qi_theta
                                      %       dqi_x
                                      %       dqi_y
                                      %       dqi_z
                                      %       dqi_theta] 

% Mld.deltaEp = zeros(Mld.nSplineSamples,1);
[T, Y]      = ode15s(@ode_nested,tspan,y0,OPTIONS);
% Q  = [q1(u); q2(u); q3(u); q4(u)]


% we must use a nested function to pass all the arguments
function dy = ode_nested(t,y)
    
    Fex = zeros(Mld.nControlPoints,4);
    
    for f1=1:Mld.nControlPoints
        for f2=1:4
            Fex(f1,f2) = interp1(Tm,Fext{f1,f2} ,t,'linear');
        end
    end
    
    dy = ODE(y,Mld,Fex,E);
end
end % simulate

