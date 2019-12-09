% Dynamic system
% MA = F + P = FP
% solution of the system framework
% A = M-1 . FP

% including the constraints (world interaction)
%  M L'       A          F + P
%(     ) (         ) = (       )
%  L 0     -Lambda          E
% where: M is matrix containing matrices 4x4
%        A is a vector of second derivative of control points
%        F applied applied force F on the point q of the spline provides
%          generalized forces Fi
%        Lambda are the Lagrangemultipliers which correspond to the force required
%          to maintain the constraints
%        L is a matrix defined using the different constraints
%          ? relatively to all degrees of freedom
%        E is a vector coding the desirated behavior of the
%          constraint, position or orientation
%
% we use Lagrangian multipliers:they allow us to set the position or the direction
% of any point of the one-dimensional object

% Solution of the system framework considering the constraints
%  M L'       A          F + P
%(     ) (         ) = (       )
%  L 0     -Lambda          E    
%
% M.A + L' (-Lambda) = F + P = FP
% M.A = FP + L'.Lambda
% and
% L.A = E
% we decopose A into At: tendency expressing the acceleration without constraints 
%                and Ac: correction due to constraints
% M(At+Ac) = FP + L'.Lambda
% M.At = FP
% M.Ac = L'.Lambda
% and
% L(At + Ac) = E == > L.Ac = E - L.At
% ==> 
%
% At = Mg-1 . FP
% Ac = Mg-1 . L'.Lambda
% L.Mg-1.L'.Lambda = E - L.At
% LML.Lambda = E - L.At

%%% dimensions of the framwork
% M (nxn) Mg (4nx4n)
% Ax, FPx, Ay,.. (nx1)
% A, At, Ac (4nx1)
% Lx, Ly,.. (cxn); c number of constraints (for a single point constraint c=1)
% L (cx4n)
% E (cx1)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state variables 
%   position of joints 
%   x1, x2, .. ,xn 
%   velocity of joints
%   xn+1, xn+2, .., xn+n
function dy = ODE(y,Mld,Fex,E)

    % state variables q(u) = [q1(u); q2(u); q3(u); q4(u)]
    qin   = y(1:length(y)/2);          
    dqin  = y(length(y)/2+1:end);
    qi    = vec2mat(qin,Mld.nControlPoints);       
    dqi   = vec2mat(dqin,Mld.nControlPoints); 
    
    %r = Mld.b'*qi(1:3,:)';
    %Theta = Mld.b'*qi(4,:)';
    
    dr = Mld.db'*qi(1:3,:)';
    dTheta = Mld.db'*qi(4,:)';
    
    ddr = Mld.ddb'*qi(1:3,:)';
    %ddTheta = Mld.ddb'*qi(4,:)';
    
    dddr = Mld.dddb'*qi(1:3,:)';
    %dddTheta = Mld.dddb'*qi(4,:)';
    
    Mld = f_dlo(Mld,dr,ddr,dddr,dTheta);
%     
%    Fc  = zeros(Mld.nControlPoints,4);
%     Fc(1,1) = (0-qi(1,1))*1000;
%     Fc(Mld.nControlPoints,1) = (2-qi(1,Mld.nControlPoints))*1000;
    
    Ps = FS(Mld,dr);
    %Ks = stretchingMatrix(Mld,dr);
    Pt = FT(Mld,dr);
    
    Pb = FB(Mld,dr);
    %Kb = bendingMatrix(Mld,dr);
    P  = Ps + Pt + Pb;
    %Mld.K = Ks + Kb;
    FP = Fex + P - Mld.Bt *dqi';               % added damper        
    FP = FP(:) + Mld.Fg;                            % [FPx,FPy,FPz,FPtheta] + gravity
    %v = dqi';
    %FP = FP - Mld.K *0.0001*v(:);
    Mld.FP = FP;
    %F_  = (Fex + P- Mld.Bt *dqi')';
    %FP_ = F_(:);

    % solve the system
    % A = M\(Fex + P); % inv(M)*(Fex + P);
    % for reducing the computation time, Mg-1 can be pre-computed 
    % ===========================================================
    %At=pinv(Mld.Mg_)*FP_; 
    %if det(Mld.Mg) ==0
        %At=pinv(Mld.Mg)*FP;  
    %else
    At=Mld.pinvMg*FP;
    %end
    %%%%%%% At_ and At should be the same where At =[Ax;Ay;Az;Atheta] 
    %                               and   At_ = [Ai;Ai+1;..] ; Ai = [Ax(i);Ay(i);Az(i);Atheta(i)]   
    if Mld.nC == 0
        Ac = zeros(4*Mld.nControlPoints,1);
    else
        Lambda = Mld.LML_inv * (E - Mld.Lg * At);
        %if det(Mld.Mg) ==0
          %  Ac=Mld.ML*Lambda;  
       % else
        Ac=Mld.pinvMg*(Mld.Lg'*Lambda);
       % end   
    end

	dy       = zeros(8*Mld.nControlPoints,1);   % 2*4 states
    % first-order derivatives
    dy(1:4*Mld.nControlPoints)          = dqin; 
    % second-order derivatives
    A = At + Ac; 
    dy(4*Mld.nControlPoints+1:end)      = A; 
        
end