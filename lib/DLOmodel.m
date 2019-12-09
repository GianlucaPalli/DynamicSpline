% Class hierarchy 
% Simulation > Model > DLO
%  Material Properties: 
% =====================
%   stiffness
% 	dissipation
% 	staticFriction
% 	dynamicFriction
% 	viscousFriction

%  Section properties:
% ======================
% - the outer perimeter of the section
% - the area
% - the axial moments of inertia Ix and Iy given by the integrals Ix= sum(x^2 dA) and Iy= sum (y^2dA)
% - the polar moment Io = sum r^2 dA = sum (x^2 + y^2) dA = Ix + Iy
% - stiffness module to the torsion J = A^2/(4pi^2(Ix+Iy)).

classdef DLOmodel
properties
    % variables 
    % material properties.
    E                       % young modulus in streching, twisting and bending [s t b], vec3
    Bt                      % damping coefficient
    sigma_E                 % yield stress 
    esp_E                   % yield strain
    FApos                   % positive yield force 
    FAneg                   % negative yield stress
    FBpos                   % positive break point stress
    FBneg                   % negative break point stress
    EApos                   % positive yield strain
    EAneg                   % negative yield strain
    EBpos                   % upper limit of break point strain
    EBneg                   % lower limit of break point strain
    K                       % stiffness 
    V                       % poison's ratio
    G                       % shear modulus (transversal elasticity modulus)
    rho                     % volume density
    lambda                  % Lam�'s first parameter
    du                      % displacement, u is between 0 and ? lenght 
    ds                      % displacement, arc length is denoted by s.
    nSplineSamples          % number of segments
    nControlPoints          % number of control points 
    order                   % psline order
    L                       % wire length
    u                       % initial wire segments
    ui                      % wire segments
    U                       % knot vector
    dU                      % step
    % section properties
    R                       % radius of wire section
    R1                      % radius of wire section
    R2                      % radius of wire section
    D                       % Diameter (circular shape)
    A                       % area
    P                       % perimeter
    Ix                      % axial moments of inertia                        
    Iy                      % axial moments of inertia 
    Io                      % the polar moment
    J                       % stiffness module to the torsion
    JM                      % inertia matrix
    Mg                      % Mg is the generalized mass matrix of the system
    Mg_                     % another form of Mg used for a test of the script
    pinvMg
    Lg                      % constraint matrix
    ML                      % constant matrix: Mg-1*L';
    LML                     % constant matrix: L*ML;
    LML_inv                 % constant matrix: LML-1;
    H                       % Hooke matrix
    Fg                      % gravity force
    FP
    ex                      % excentricity of ellipse 
    % force
    f                       %  force
    % constraints
    nC                      % number of constraints 
    % initial coordinates
    r0                      % (x,y,z)
    Theta0                  % roll
    dr0                     % first derivative of r
    dTheta0                 % first derivative of theta
    ddr0                    % second derivative of r
    ddTheta0                % second derivative of theta
    dddr0                   % third derivative of r
    dddTheta0               % third derivative of theta
    % strains 
    e                       % strain vector
    e0                      % initial strain vector
    es                      % streching strain
    et                      % twisting strain
    eb                      % bending strain
    es0                     % initial streching strain
    et0                     % initial twisting strain
    eb0                     % initial bending strain
    deltaEp                 % plastic strain offset
    Ee                      % relative deformation
    Eep                     % relative deformation considering elasticity and plasticity 
    % other properties
    tao                     % torsion (geometrical or Frenet twisting)
    k                       % curvator
    % compactness variables
    C
    PI
    TI
    % stress
    segma_ideal
    sigma
    % other parameters
    g                       % gravity 
    m                       % dimensionless constant
    % Splines
    % taoParam            % tension
    BSpline                 % structure info of Bspline
    p                       % degree of splines
    DoF                     % degree of freedom
    BasisFun                % calculated basis functions
    b                       % basis functions
    bFilt                   % zero-phase filtered basis functions
    nd                      % derivative degree
    db                      % first derivative
    ddb                     % second derivative 
    dddb                    % third derivative 
    dbFilt                  % zero-phase filtered first derivative
    ddbFilt                 % zero-phase filtered second derivative 
    dddbFilt                % zero-phase filtered third derivative     
    sectionType
end % properties
methods
    % constructor
    function F = DLOmodel()
        % default properties
        F.R1              = 0;
        F.R2              = 0;
        F.ex              = 0;
        F.E               = 120.7; 
        F.Bt              = 10;
        F.sigma_E         = 100;
        F.FApos           = 100;
        F.FAneg           = -100;
        F.FBpos           = 300;
        F.FBneg           = -300;
        F.V               = 0.35;
        F.rho             = 2.7;
        F.du              = 0.05;
        F.L               = 2;
        F.g               = -9.81;
        F.m               = 20;
        F.p               = 3;
        F.nd              = 3;
        F.nC              = 0;         
        % F.taoParam        = 1/2; % this values is used for Catmull-Rom spline
        % F.nDeg            = 3;
        F.DoF             = 4;
        F.dU              = 0.2; 
        % F.halfspaceNormal=[1;0;0];
        F.sectionType = 'circular';
    end
    % set the main model parameter values
    function F=setModel(F,sectionType,R1,R2,E,Bt,V,rho,FApos,FAneg,FBpos,FBneg,du,L,m,p,nd,nC)
        F.E=E;
        F.Bt=Bt;
        F.V=V;
        F.rho=rho;
        F.FApos=FApos;
        F.FAneg=FAneg;
        F.FBpos=FBpos;
        F.FBneg=FBneg;
        F.sigma_E=FApos;        
        F.du = du;
        % F.U  = U;
        F.L  = L;
        F.m  = m;
        F.p  = p;
        F.nd = nd;
        F.nC = nC;
        F.sectionType = sectionType;
        switch sectionType
            case 'circular'
                F.R=R1;
            case 'tubic'
                F.R1=R1;
                F.R2=R2;
            case 'elleptic'
                F.R1=R1;
                F.R2=R2;   
            case 'square'
                F.R=R1;
            case 'rectangular'
                F.R1=R1;
                F.R2=R2;
            case 'I-Beam'
                F.R1=R1; % vec2
                F.R2=R2; % vec2
        end
        F=calcParameters(F,sectionType);
    end
    
    %%%%%%% constants  %%%%%%%
    function F=calcParameters(F,sectionType)
        %      
        switch sectionType
            case 'circular'
                F.D= 2*F.R;
                F.P= pi*F.D;
                F.A= pi*F.D^2/4;
                F.Ix= pi*F.D^4/64;
                F.Iy= F.Ix;
                F.Io= pi*F.D^4/32;
                F.J= F.Io;
            case 'tubic'
                F.D= 2*F.R1;
                d= 2*F.R2;
                F.P= pi*F.D;
                F.A= pi*(F.D^2-d^2)/4;
                F.Ix= pi*(F.D^4-d^4)/64;
                F.Iy= F.Ix;
                F.Io= pi*(F.D^4-d^4)/32;
                F.J= F.Io;
            case 'elleptic'
                F.P= sqrt(2*pi*sqrt(F.R1^2+F.R2^2));
                F.A= pi*F.R1 * F.R2;
                F.Ix= (pi/4)*F.R1 * F.R2^3;
                F.Iy= (pi/4)*F.R1^3 * F.R2;
                F.Io= F.Ix + F.Iy;
                F.J= pi*(F.R1^3 * F.R2^3)/(F.R1^2 + F.R2^2);
                F.ex= sqrt(1-(F.R2^2)/(F.R1^2));
            case 'square'
                F.P= 8*F.R;
                F.A= 4*F.R^2;
                F.Ix= 4*F.R^4/3;
                F.Iy= F.Ix;
                F.Io= F.Ix + F.Iy;
                F.J= 24*F.R^4/(pi^2);
            case 'rectangular'
                F.P= 4*(F.R1+F.R2);
                F.A= 4*F.R1*F.R2;
                F.Ix= 4*F.R1*(F.R2^3)/3;
                F.Iy= 4*(F.R1^3)*F.R2/3;
                F.Io= F.Ix + F.Iy;
                F.J= (24*(F.R1^3)*(F.R2^3))/((pi^2)*((F.R1^2)+(F.R2^2)));
            case 'I-Beam' % check
                B= 2*F.R2(1);
                h= 2*F.R1(1);
                t= 2*(F.R1(1)-F.R1(2));
                w= 2*(F.R2(1)-F.R2(2));
                F.P= 2*h + 4*B - 2*w;
                F.A= 2*B*t + w*h;
                F.Ix=(B*h^3-((B-w)*(h-2*t)))/12;
                F.Iy= B^3*t/6 + w^3*(h-2*t)/12;
                F.Io= F.Ix + F.Iy;
                F.J= F.A^4/(4*pi^2*F.Io);                  
        end % switch
        %
        F.esp_E = F.sigma_E./F.E;           % yield strain = yield stress/ young modulus
        F.EApos = F.FApos./F.E;
        F.EAneg = F.FAneg./F.E;
        F.EBpos = F.FBpos./F.E;
        F.EBneg = F.FBneg./F.E;
        F.G = F.E./(2*(1+F.V));            % paper: Geometrically exact dynamic splines
        F.lambda= F.E.*F.V/((1+F.V)*(1-2*F.V)); % (NOT used: non-perfect plasticity characterestic)
        F.u = 0:F.du:F.L;
        % F.nSplineSamples = length (F.u);        
        F.JM = inertiaMat(F);
        F.H  = Hooke(F);
        % Compute first initial frame, then propagate it along
        % the spline (eq. 29)
    end

    function dataStore = getContactForceInfo(F)
        dataStore = F;
    end

    %%%%%% %%%%% %%%%%%% %%%%%%%
    %%%%%%   Algorithm    %%%%%%
    %%%%%% %%%%% %%%%%%% %%%%%%%
    
    function F=f_dlo(F,dr,ddr,dddr,dTheta)
        % in elastic and plastic regions
        % Book: Plasticity theory_Lubliner2005
        % [F.sigma, F.segma_ideal] = stress_strain(F,eps);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% r(VECnx3) = (x,y,z), theta(VECnx1), bi(VECnxi) %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        F.ds = (vecnorm(dr')*F.dU)';
        
        F.es = 1-vecnorm(dr')';
        
        %%% Plasticity and break point algorithm
        for i=1:F.nSplineSamples % maybe needs to be changed
        % strains     
            %F.ds(i,1) = displacement(F,dr(i,:));
            % streching
            %F.es(i,1) = strechDeform(F,dr(i,:));                                                      
            % torsion
            [F.et(i,1), F.tao(i,1),F.C(i,:)] = torsionDeform(F,dr(i,:),ddr(i,:),dddr(i,:),dTheta(i));
            % curvator
            [F.eb(i,1),F.k(i,1)] = bendDeform(F,dr(i,:),ddr(i,:)); 
       
            F.e(i,:)  = [F.es(i) F.et(i) F.eb(i)];
            
            %%% d(r')/dr1 = b1'; d(r')/dr2 = b2'; d(r')/dr3 = b3'
            %%% d(r'')/dr1 = b1''; d(r'')/dr2 = b2''; d(r'')/dr3 = b3''
            drdri  = [F.db(:,i), F.db(:,i),F.db(:,i)]; % F.db(i,1:3);
            ddrdri = [F.ddb(:,i), F.ddb(:,i),F.ddb(:,i)];
            %%% dC/dri =  d(r' x r'')/dri
            %%% dC/dri = d(r')/dri x r'' + r' x d(r'')/dri
            
            for j = 1:F.nControlPoints
                F.PI(i,:,j) =  cross(drdri(j,:),ddr(i,:)) + cross(dr(i,:),ddrdri(j,:));
                F.TI(i,:,j) = F.C(i,:).*F.dddb(j,i)'-cross(F.PI(i,:,j),dddr(i,:)) - 2*F.tao(i)*cross(F.C(i,:),F.PI(i,:,j));
            end
            %[F.PI{i} ,F.TI{i}]=PT(F,drdri,ddrdri,dr(i,:),ddr(i,:),dddr(i,:),i); 
            
        % for all spline samples
            F.Ee(i,:) = F.e(i,:) - F.e0(i,:);       % relative deformation
            F.Eep       = F.Ee(i,:)- F.deltaEp(i,:);
            % EApos,EAneg,EBpos,EBneg are vectors, 
            % for each compenent, there is a certain characteristic 
            % Compute positive and negative deformation energy
            Epos      = F.Eep - F.EApos;    
            Eneg      = F.Eep - F.EAneg;
            
            %% Evaluate plasticity for each control point
            for ie=1:length(F.Eep)
                if Epos(ie) > 0
                    F.deltaEp(i,ie) = F.deltaEp(i,ie) + Epos(ie);
%                     if F.Ee(i,ie) > F.EBpos(ie)
%                         disp('fracture');
%                         % simulate the fracture.
%                     end
                elseif Eneg(ie) < 0
                    F.deltaEp(i,ie) = F.deltaEp(i,ie) + Eneg(ie);
%                     if F.Ee(i,ie) < F.EBneg(ie)
%                         disp('fracture');
%                         % simulate the fracture.
%                     end
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%      
    end

    % Hooke matrix
    function H = Hooke(F)
        % [s t b]
        H=pi*F.D^2/4*[F.E(1) 0 0;
                        0 F.G(2)*F.D^2 0;
                        0 0 F.E(3)*F.D^2/16];
%         H=pi*F.D^2/4*[F.E 0 0;
%                         0 F.G*F.D^2 0;
%                         0 0 F.E*F.D^2/16];                    
    end
    function ds = displacement(F,dr)
        % ds = sqrt(sum(dr.^2,1))*F.du;
        ds = norm(dr)*F.dU;
    end
    % strain energy considering the plasticity
    function U = strainEnergy(F)
        % H = Hooke(F);
        % e: state variable
        U = 1/2 *trapz ((F.e - F.e0 - F.deltaEp)'*F.H *(F.e  - F.e0 - F.deltaEp));
        % U = 1/2 *SMn *(F.e  - F.e0 - F.deltaEp)'*F.H *(F.e  - F.e0 - F.deltaEp)*F.ds;
    end
    function Es = strechDeform(F,dr)
        Es = 1 - norm(dr);
    end
    function [Et,tao,C] = torsionDeform(F,dr,ddr,dddr,dTheta)
        C  = cross(dr,ddr);
        if norm(C) < 1e-8
            B = pinv(C); % B = C/||C||^2 = 1/C
        else
            B = C/((norm(C))^2);
        end
        tao= dot(B,dddr);    
        Et = dTheta + tao;
    end
    function [Eb,k]= bendDeform(F,dr,ddr)
        normC  = norm(cross(dr,ddr));
        if normC < 1e-8
            k  = 0; 
        else
            k  = normC/((norm(dr))^3);   
        end
        Eb   = k; 
    end
    function FFF = forces(F)
        % vector F as the data of FS, FT and FB
        % H = Hooke(F);
        FFF = F.H*(F.e  - F.e0 - F.deltaEp);
    end
    % stretching force
    function Fs = FS(F,dr)
        % [s t b]
        fr = zeros(F.nControlPoints,3);
        norms=sqrt(sum(F.dr0.^2,2))./sqrt(sum(dr.^2,2));
        if isnan(norms(1))
            norms(1) = 1;
        end
        for i=1:F.nControlPoints    % to be replaced with something simpler 
            if F.sectionType == 'circular'
                fr(i,:) = -pi*F.E(1)*F.D^2/4 *trapz(F.ui,(1.-norms).*F.db(i,:)'.*(dr));
                % fr(i,:) = -pi*F.E*F.D^2/4 *SMn*(1.-norms)*(dr)'*F.db(i,:)'*F.ds(i);
            else
                fr(i,:) = -F.E(1)*trapz(F.ui,(1-norms).*F.db(i,:)'.*dr);
                % fr(i,:) = -F.E*SMn*(1-norms)*dr*F.db*F.ds;
            end
        end
        % Since stretching strain energy Us does not depend on it
        ft= zeros(F.nControlPoints,1);
        Fs=[fr ft];
    end
     function Ks = stretchingMatrix(F,dr)
        Ks = zeros(4*F.nControlPoints,4*F.nControlPoints);
        norm_dr2 = sum(dr.^2,2);
        norms=sqrt(sum(F.dr0.^2,2))./sqrt(norm_dr2);
%         if isnan(norms(1))
%             norms(1) = 1;
%         end
        for i=1:F.nControlPoints    % to be replaced with something simpler 
            for j =1:F.nControlPoints
                for k = 1:3
                    for l = k:3
                        if F.sectionType == 'circular'
                            if  k==l                        
                                Ks(i+(k-1)*F.nControlPoints,j+(l-1)*F.nControlPoints)...
                                    = pi*F.E(1)*F.D^2/4 *trapz(F.ui,(1+dr(:,k).*norms./norm_dr2-norms).*F.db(i,:)'.*F.db(j,:)');
                            else
                                Ks(i+(k-1)*F.nControlPoints,j+(l-1)*F.nControlPoints)...
                                    = pi*F.E(1)*F.D^2/4 *trapz(F.ui,dr(:,k).*dr(:,l).*norms./norm_dr2.*F.db(i,:)'.*F.db(j,:)');
                                Ks(i+(l-1)*F.nControlPoints,j+(k-1)*F.nControlPoints) = Ks(i+(k-1)*F.nControlPoints,j+(l-1)*F.nControlPoints);
                            end
                            % fr(i,:) = -pi*F.E*F.D^2/4 *SMn*(1.-norms)*(dr)'*F.db(i,:)'*F.ds(i);
                        else
                            %fr(i,:) = -F.E(1)*trapz(F.ui,(1-norms).*F.db(i,:)'.*dr);
                            % fr(i,:) = -F.E*SMn*(1-norms)*dr*F.db*F.ds;
                        end
                    end
                end
            end
        end
        
     end
    
    % twisting force
    function Ft =FT(F,dr)
        % [s t b]
        fr = zeros(F.nControlPoints,3);
        ft = zeros(F.nControlPoints,1);
        for i=1:F.nControlPoints    % to be replaced with something simpler 
            if F.sectionType == 'circular'
                % epsi_t0 = epsi_t(1);
                normC = norm(F.C(i,:));
                if normC < 1e-8
                    B = 0;
                else
                    B = 1/(normC^2);
                end
                % fr(i,:) = -pi*F.G*F.D^4/32 *SMn*(F.et-F.et0)*F.TI*B*F.ds(i);
                fr(i,:) = -pi*F.G(2)*F.D^4/32 *trapz(F.ui,(F.et-F.et0- F.deltaEp(:,2)).*F.TI(:,:,i)*B);
                % roll contribution
                if norm(dr(i,:)) ==0
                    ft(i) = 0;
                else
                    ft(i) = -pi*F.G(2)*F.D^4/64 *trapz(F.ui,(F.et-F.et0- F.deltaEp(:,2)).*(F.db(i,:)'.*1./sqrt(sum(dr.^2,2)))); % 64 or 32????
                    % ft(i) = -pi*F.G*F.D^4/64 *SMn*(F.et(i)-F.et0(i))*F.db(4,i)/norm(dr(i,:))*F.ds(i); % 64 or 32????
                end
            else
    %             fr = ;
    %             ft = ;            
            end
        end
        Ft=[fr ft];
    end
    
    % bending force
    function Fb =FB(F,dr)
        % [s t b]
        % bending strain energy Ub does not depend on 
        fr = zeros(F.nControlPoints,3);
        for i=1:F.nControlPoints    % to be replaced with something simpler 
            if F.sectionType == 'circular'
                if sqrt(sum(F.C.^2,2)) < 10^-8 % F.C < 10^-8
                    % if norm(dr(i,:)) == 0
                        %%% fr(i,:) = -pi*F.E*F.D^4/64 *F.eb0(i)*SMn(i)* cross(F.PI(i,:),ones(3,1))/(1)*F.ds(i);
                        % fr(i,:) = -pi*F.E*F.D^4/64 *F.eb0*trapz( cross(F.PI,ones(F.nSplineSamples,3))/(1));
                    % else
                    %fr(i,:) = -pi*F.E(3)*F.D^4/64 *trapz(F.ui, (F.eb-F.eb0- F.deltaEp(:,3)).*1./((sqrt(sum(dr.^2,2))).^2).* (1./(sqrt(sum(F.C.^2,2)).*sqrt(sum(dr.^2,2))).*cross(F.C,F.PI(:,:,i))-3*(F.k.*F.db(i,:)').*dr));
                    fr(i,:) = -pi*F.E(3)*F.D^4/64 *trapz(F.ui,(F.eb0).*(1./((sqrt(sum(dr.^2,2))).^3)).* cross(F.PI(:,:,i),ones(F.nSplineSamples,3)));
                        % fr(i,:) = -pi*F.E*F.D^4/64 *F.eb0(i)*SMn(i)* cross(F.PI(i,:),ones(3,1))/((norm(dr(i,:)))^3)*F.ds(i);
                    % end
                else
                    fr(i,:) = -pi*F.E(3)*F.D^4/64 *trapz(F.ui, (F.eb-F.eb0- F.deltaEp(:,3)).*1./((sqrt(sum(dr.^2,2))).^2).* (1./(sqrt(sum(F.C.^2,2)).*sqrt(sum(dr.^2,2))).*cross(F.C,F.PI(:,:,i))-3*(F.k.*F.db(i,:)').*dr));
                    % fr = -pi*F.E*F.D^4/64 *SMn(i)* (F.eb(i)-F.eb0(i))/(norm(dr(i,:))^2) * (cross(F.C(i,:),F.PI(i,:))/(norm(F.C(i,:))*norm(dr(i,:)))*3*F.k(i)*F.db(1:3,i)*dr(i,:))*F.ds(i);
                end
            else
    %             fr = ;           
            end
        end
        ft = zeros(F.nControlPoints,1);
        Fb=[fr ft];
    end    
    function Kb = bendingMatrix(F,dr)
        Kb = zeros(4*F.nControlPoints,4*F.nControlPoints);
        norm_dr2 = sum(dr.^2,2);
        %norms=sqrt(sum(F.dr0.^2,2))./sqrt();
%         if isnan(norms(1))
%             norms(1) = 1;
%         end
        for i=1:F.nControlPoints    % to be replaced with something simpler 
            for j =1:F.nControlPoints
                for k = 1:3
                    for l = k:3
                        Gi = cross(F.C,F.PI(:,:,i));
                        Gj = cross(F.C,F.PI(:,:,j));
                        if F.sectionType == 'circular'
                            if  k==l                                                        
                                index = 1:3;
                                index(k) = [];                        
                                Kb(i+(k-1)*F.nControlPoints,j+(l-1)*F.nControlPoints)...
                                    = pi*F.E(3)*F.D^4/64 *trapz(F.ui,(F.PI(:,index(1),i).*F.PI(:,index(1),j)+F.PI(:,index(2),i).*F.PI(:,index(2),j))./norm_dr2.^3 ...
                                    -6*(Gi(:,k).*F.db(j,:)'+Gj(:,k).*F.db(i,:)').*dr(:,k)./norm_dr2.^4 ...
                                    +3*(F.k.*F.db(i,:)'.*F.db(j,:)').*(8*dr(:,k).^2./norm_dr2-1)./norm_dr2);
                            else
                                index = 1:3;
                                index([k l]) = [];                       
                                %index(l) = [];
                                Kb(i+(k-1)*F.nControlPoints,j+(l-1)*F.nControlPoints)...
                                    = pi*F.E(3)*F.D^4/64 *trapz(F.ui,-(F.PI(:,k,i).*F.PI(:,l,j)+Gj(:,index(1)).*(F.ddb(j,:)'.*F.db(i,:)'-F.db(j,:)'.*F.ddb(i,:)'))./norm_dr2.^3 ...
                                    -6*(Gi(:,k).*F.db(j,:)'.*dr(:,l)+Gj(:,l).*F.db(i,:)'.*dr(:,k))./norm_dr2.^4 ...
                                    +24*F.k.*F.db(i,:)'.*F.db(j,:)'.*dr(:,k).*dr(:,l)./norm_dr2.^2);
                                Kb(i+(l-1)*F.nControlPoints,j+(k-1)*F.nControlPoints) = Kb(i+(k-1)*F.nControlPoints,j+(l-1)*F.nControlPoints);
                            end
                            % fr(i,:) = -pi*F.E*F.D^2/4 *SMn*(1.-norms)*(dr)'*F.db(i,:)'*F.ds(i);
                        else
                            %fr(i,:) = -F.E(1)*trapz(F.ui,(1-norms).*F.db(i,:)'.*dr);
                            % fr(i,:) = -F.E*SMn*(1-norms)*dr*F.db*F.ds;
                        end
                    end
                end
            end
        end
        
    end
    
    function [t,n,b] = Frenet(F,dr)
        % Tangent vector of the curvature 
        t = dr/norm(dr);
        % normal vector of the curvature (principal normal vector)
        n = dt/F.k;
        % binormal vector
        b = (dn + F.k*t)/F.tao;
    end
    % constrains on a point
     function L = positionConstraints(F,u0)
        % this matrix is for each generalized coordinate  
        % u0 should be the index of u0
        L=cell(F.nControlPoints);
        for i=1:F.nControlPoints
            B=F.b(i,u0);
            L{i} = B*eye(4);
        % if we want to constraint one or more coordinates, we can
        % reduce the matrix accordingly 
%             switch constraintOn
%                 case 'all'
%                 case 'x'
%                 case 'y'
%                 case 'z'
%                 case 'theta'
%                 case 'xy'
%                 case 'xz'
%                 case 'xtheta'
%                 case 'xytheta'
%                 case 'xztheta'
%                 case 'yz'
%                 case 'ytheta'
%                 case 'yztheta' 
%                 case 'ztheta' 
%             end            
        end
    end
    function L=orientationConstraints(F,u0) 
        % this matrix is for each generalized coordinate ri
        L=cell(F.nControlPoints);
        for i=1:F.nControlPoints
            B=F.db(i,u0);
            L{i} = B *eye(3);                         
        end        
    end
    function L = distanceConstraints(F,u0,r,p) 
        % this constraint g defines the distance d between the position r
        % and a point p such that g(r,u)=(r(u)-p)^2 - d^2 = 0
        % for each generalized coordinate ri
        L=cell(F.nControlPoints);        
        for i=1:F.nControlPoints
            B=2*(r(u0,:)-p)*eye(3)*F.b(i,u0);
            L{i} = [B B B 0];                         
        end    
    end
    function L = planConstraints(F,u0,n) 
        % this constraint g defines between the position r
        % and a plan defined by its normal n such that g(r,u)=(r(u)-r0(u)).n = 0
        % for each generalized coordinate ri
        L=cell(F.nControlPoints);        
        for i=1:F.nControlPoints
            L{i} = [n(1)*F.b(i,u0) n(2)*F.b(i,u0) n(3)*F.b(i,u0)];    % n*bi                     
        end    
    end
    function L = axisConstraints(F,u0,n1,n2) 
        % this constraint g defines between the position r
        % and an axis defined by two normals n1 and n2 
        % such that g1(r,u)=(r(u)-r0(u)).n1 = 0
        %           g2(r,u)=(r(u)-r0(u)).n2 = 0
        % for each generalized coordinate ri
        L=cell(F.nControlPoints);        
        for i=1:F.nControlPoints
            L{i} = [n1(1)*F.b(i,u0) n1(2)*F.b(i,u0) n1(3)*F.b(i,u0);
                    n2(1)*F.b(i,u0) n2(2)*F.b(i,u0) n2(3)*F.b(i,u0)];                        
        end    
    end
    function [Li, Lu]= slidingPointConstraints(F,u0,u,dr,c)
        % for each generalized coordinate ri
        % c is viscous friction coefficient
        Li=cell(F.nControlPoints);
        for i=1:F.nControlPoints
            B=F.b(i,u0);
            Li{i} = [B 0 0 0;
                     0 B 0 0;
                     0 0 B 0;
                     0 0 0 0];                  
        end
        Lu = [0       0       0       dr(u,1);
              0       0       0       dr(u,2);
              0       0       0       dr(u,3);
              dr(u,1) dr(u,2) dr(u,3) c     ];         
    end
    function [Li, Lu]= slidingRotatingPointConstraints(F,u0,u,dr_ref,c)
        % for each generalized coordinate ri
        % c is viscous friction coefficient
        Li=cell(F.nControlPoints);
        for i=1:F.nControlPoints
            B =F.b(i,u0);
            dB=F.db(i,u0);
            Li{i} = [B 0 0;
                     0 B 0;
                     0 0 B;
                     0 0 0; 
                     dB 0 0;
                     0 dB 0;
                     0 0 dB];                   
        end 
        Lu    = [0           0           0           dr_ref(u,1) 0 0 0;
                 0           0           0           dr_ref(u,2) 0 0 0;
                 0           0           0           dr_ref(u,3) 0 0 0;
                 dr_ref(u,1) dr_ref(u,2) dr_ref(u,3) c           0 0 0;
                 0           0           0           0           0 0 0;
                 0           0           0           0           0 0 0;
                 0           0           0           0           0 0 0];        
    end 
    function Lu= slidingAbscissaConstraints(F,u,dr,c)
        % a is an index
        Lu = [0       0       0       dr(u,1);
              0       0       0       dr(u,2);
              0       0       0       dr(u,3);
              dr(u,1) dr(u,2) dr(u,3) c];                  
    end
    %%% Constraints on several points
    % constraints of relative positions
    function L = relativePositionConstraints(F,u0,u1) 
        % this matrix is for each generalized coordinate and the rotation
        % angle
        L=cell(1,F.nControlPoints);
        for i=1:F.nControlPoints
            B=F.b(i,u0)-F.b(i,u1);
            L{i} = B*eye(4);                       
        end
    end
    % constraints of relative orientations
    function L=relativeOrientationConstraints(F,u0,u1) 
        L=cell(F.nControlPoints);
        for i=1:F.nControlPoints
            B=F.db(i,u0)-F.db(i,u1);
            L{i} = B*eye(3);                         
        end        
    end
    % constraints of relative coplanar embedding
    function L=relativeCoplanarEmbeddingConstraints(F,u0,u1,alpha0) 
        % u0, u1: index of u, .e.g uo -> a = 0
        L=cell(F.nControlPoints);
        for i=1:F.nControlPoints
            B=F.b(i,u0)-F.b(i,u1)-alpha0*F.db(i,u0);
            L{i} = B*eye(3);                         
        end        
    end 
     % constraints of relative distance 
    function L=relativeDistanceConstraints(F,u0,u1,r) 
        % u0, u1: index of u, .e.g uo -> a = 0
        L=cell(F.nControlPoints);
        for i=1:F.nControlPoints
            B=2*(F.b(i,u0) - F.b(i,u1));
            L{i} = B*[r(u0,1)-r(u1,1) r(u0,2)-r(u1,2) r(u0,3)-r(u1,3)];                         
        end        
    end 
    function L = barycentricConstraints(F,a,location)
        % for modeling the form of meshes, Nourrit et al. used spring
        % networks for keeping the entanglement configuration. This is not
        % convienent because of the lack of stability about the
        % equilibratium state and the difficulty to choose an adaguate
        % stiffness. 
        % This method propose the use of barycentric constraints which can
        % be generalized d degrees of freedom q(uj) of related constrained.
        % This allows to easily impose a pattern.
        % To Violate a the imposed constraint on the displacement of a barycenter:
        % translation for the positions q(uj), rotation for the orientations dq(uj). 
        % if G=q(ud), the barycenter of a set of d-1 points q(uj) and
        % points aj
        % the constraint is defined by: g(q(uj))= sum (aj q(uj) - q(ud))= 0
        % where G= q(ud). If G is the center of gravity, all aj = 1
        %                 if G is not situated on a spline, S = 0
        % G = q(ud) = {(q(uj),aj)}
        
        L=cell(F.nControlPoints);
        for i=1:F.nControlPoints
            switch location 
                case 'out'
                    bi_ud=0;
                case 'in'
                    bi_ud = F.b(i,F.nSplineSamples); % F.b(i,F.nSplineSamples-1)
                case 'center of gravity'
                    bi_ud = F.b(i,F.nSplineSamples);
                    a=ones(1,F.nSplineSamples);
            end
            B= sum(a(1:F.nSplineSamples-1).*F.b(i,1:F.nSplineSamples-1) - bi_ud);
            L{i} = B*eye(4);                         
        end         
    end


     
    function [Yo, U, F]= initialization(F,y0)
        % y0 = [r0,Theta0,dr0,dTheta0];

        intervals = F.nControlPoints - 6 + F.order;
        F.ui = linspace(0,F.L,F.nSplineSamples); 
        
        F.dU = F.ui(2)-F.ui(1);
        
        knots = augknt(linspace(0,F.L,intervals),F.order);
        
        U = knots;
        
        F.U = U;
        
        colmat = spcol(knots,F.order,brk2knt(F.ui,4));
        
        F.b = colmat(1:4:end,:)';
        F.db = colmat(2:4:end,:)';
        F.ddb = colmat(3:4:end,:)';
        F.dddb = colmat(4:4:end,:)';
        
        x0 = linspace(0,F.nControlPoints,F.nControlPoints);
        y0 = zeros(1,F.nControlPoints);
        z0 = zeros(1,F.nControlPoints);
        theta0 = zeros(1,F.nControlPoints);

        interp_fun = colmat*[x0' y0' z0' theta0'];        
        
        P = pinv(F.b')*[F.ui' zeros(F.nSplineSamples,3)];
        
        dP = zeros(size(P));
        
        p0 = colmat*P;
        
        F.r0 = p0(1:4:end,1:3);
        F.Theta0 = p0(1:4:end,4);
        
        F.dr0 = p0(2:4:end,1:3);
        F.dTheta0 = p0(2:4:end,4);
        
        F.ddr0 = p0(3:4:end,1:3);
        F.ddTheta0 = p0(3:4:end,4);
        
        F.dddr0 = p0(4:4:end,1:3);
        F.dddTheta0 = p0(1:4:end,4);
        
        Yo = [P dP];    % dqi0 = 0 usually
               
        % calculation of initial strains: e0
        F = initialStrains(F);
        [F.Mg,F.Mg_, F.pinvMg] = calcMg(F);
    end    
    
    function JM = inertiaMat(F) % this should be in function of u
        % linear desntiy of a rode = 9 + 2 *sqrt(F.L)
        mu=F.rho*F.A;
        JM = [mu 0 0 0;
             0 mu 0 0; 
             0 0 mu 0; 
             0 0 0 F.Io];
    end
    function [Mg,Mg_, pinvMg] = calcMg(F)
        M      = cell (F.nControlPoints,F.nControlPoints);      % (nxn)
        Mx     = zeros(F.nControlPoints,F.nControlPoints);      % (nxn)
        My     = zeros(F.nControlPoints,F.nControlPoints);      % (nxn)
        Mz     = zeros(F.nControlPoints,F.nControlPoints);      % (nxn)
        Mtheta = zeros(F.nControlPoints,F.nControlPoints);      % (nxn)
        M0     = zeros(F.nControlPoints,F.nControlPoints);      % (nxn)
        for i=1:F.nControlPoints     
            for j=1:F.nControlPoints 
                % M{i,j}    = F.JM*sum(F.b(i,:).*F.b(j,:))*F.dU; % ||r'||?  
                % M{i,j}    = F.JM*cumtrapz(F.ui,F.b(i,:).*F.b(j,:));
                M{i,j}     = F.JM*trapz(F.ui,F.b(i,:).*F.b(j,:)); % ||r'||?
                % M{i,j}     = F.JM*trapz(F.b(i,:).*F.b(j,:)); 
                Mdiag      = diag(M{i,j});
                Mx(i,j)    = Mdiag(1);
                My(i,j)    = Mdiag(2);
                Mz(i,j)    = Mdiag(3);
                Mtheta(i,j)= Mdiag(4);
            end   
        end
        % generation of big matrix Mg  where theoritically Mx,My,Mz should be the same 
        Mg_ = cell2mat(M);
        Mg = [Mx M0 M0 M0;
              M0 My M0 M0
              M0 M0 Mz M0
              M0 M0 M0 Mtheta];  
        pinvMg = pinv(Mg);
%         pinvMg = [pinv(Mx) M0 M0 M0;
%               M0 pinv(My) M0 M0
%               M0 M0 pinv(Mz) M0
%               M0 M0 M0 pinv(Mtheta)]; 
    end
    function F = initialStrains(F)
        for i=1:F.nSplineSamples
            % streching
            F.es0(i,1) = strechDeform(F,F.dr0(i,:));                                                      
            % torsion
            F.et0(i,1) = torsionDeform(F,F.dr0(i,:),F.ddr0(i,:),F.dddr0(i,:),F.dTheta0(i));
            % curvator
            F.eb0(i,1) = bendDeform(F,F.dr0(i,:),F.ddr0(i,:));
        end
        F.e0 = [F.es0 F.et0 F.eb0];
        F.deltaEp = zeros(size(F.e0));
        % F.deltaEp = zeros(F.nSplineSamples,1);
    end
    % elastic relationship
    function sigma = StSt(F,miu,eps)
        sigma=2*miu*eps + F.lambda*trace(eps)*eye(3,1);
    end
    %%%% stress-strain characteristic
    function [sigma, segma_ideal] = stress_strain(F,eps)
        sigma = zeros(size(eps));
        segma_ideal = zeros(size(eps));
        for i=1:length(eps)   
            % in elastic and plastic regions
            % Book: Plasticity theory_Lubliner2005
            n = 1/F.m;
            % elastic-plastic 
            if eps(i) <=F.esp_E                   % initial yield stress (sigma_E)
                sigma(i) = F.E.*eps(i);  
                eps_e = eps(i);                   % elastic strain in elastic region
            else 
                sigma(i) = F.sigma_E *(F.E.*eps(i)/F.sigma_E)^n;
                eps_e = F.esp_E;                  % elastic strain
                eps_p = eps(i) - eps_e;           % plastic strain
            end

            % elastic-ideal plastic 
            if eps(i) <=F.esp_E                   % initial yield stress (sigma_E)
                segma_ideal(i) = F.E.*eps(i);
                eps_e = eps(i);
            else
                segma_ideal(i) = F.sigma_E;
                eps_e = F.esp_E;
                eps_p= eps(i) - eps_e;
            end
        end
    end

end % methods
end % classdef