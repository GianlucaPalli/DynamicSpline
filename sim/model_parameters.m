%% this is a model of cable attached from the two ends to fixed points and 
%  a costant force is applied on the middle  

%% Example

clear all
close all
clc

addpath ../lib

% model 
Mld = DLOmodel;
% read initial position
%load ('Pos.mat')

%% parameters 
D = 1000;                % (NOT used) weight per unit length at each point of the object 
L = 2.0;                 % wire length
% n = 50;                % n+1 control points
du = 0.5;               % lenght step "distance from one endpoint of the object"
                         % set correctly in correspondence to initial
                         % values of the coordinates
dU = 0.05;              % step  
p0 = [0 0 0 0 0 0]';     % initial pose
pf = [0.2 0 1.2 0 0 0]'; % final pose
% pf = [0 0 1.5 0 0 0]';   
floor_level = -0.3;      % plane definition
% we obtain m +1  knots for n+1 control points
%%% m = n + p +1 = 6 + 3 +1
% U = [0 0 0 0 1/4 2/4 3/4 1 1 1 1]; % knot vector
u = 0:du:L;              % wire segments 
sectionType ='circular'; % wire section shape 
m = 20;                  % dimensionless constant (related to NON-perfect plasticity) for typical stress-strain curve 
                         % not used as we considered a characteristic with
                         % perfect plasticity
R1 = 1e-3;               % radius of wire section
R2 = 0;                  % radius of wire section if not circular (see thr description of wire section)
E  = [5000 5 0.5]*1e6;      % young modulus: streching, twisting and bending [s t b]
sigma_E = 100;           % yield stress
Bt      = 5;            % added damping coefficient (I added this parameter to damp the system, it can be zero as well)
% assume the material has different characteristics in streching, twisting
% and bending (we obtain three different elastic-plastic characteristics),
% [s t b]
FApos   = [100 100 100]; % positive yield stress 
FAneg   = -FApos;        % negative yield stress
FBpos   = 3e10*[1 1 1];  % positive break point stress
FBneg   = -FBpos;        % positive break point stress
V = 0.33;                % poison's ratio
rho = 2.7e6;               % volume density of aluminum is about 2.7 g/cm3
p = 3;                   % p: degree of the spline
nd=3;                    % derivative degree of splines
nC=2;        % 2         % number of constraints 
Econst=zeros(2,1); % [0.0 0.0]'1.0 % a vector coding the intensity of the violation of the different constraints.
%% integrator settings
OPTIONS = odeset('RelTol',1e-3,'AbsTol',1e-4);
options = optimset('TolFun',1e-9,'MaxFunEvals',1e5,'MaxIter',1e4,'Algorithm','trust-region-reflective'); % Option to display output

% simulation time 
Tm  = [0 10];
% initial values
% we assume the wire is straight, aligned with x axis 
% no bending, no streching, no torsion,
% r0(0,0,0)... rn(L,0,0)
nSeg0 = length (u);
%X0     = Pos.x;
%Y0     = Pos.y;
%Z0     = Pos.z;
X0     = u';
Y0     = zeros(nSeg0,1);
Z0     = zeros(nSeg0,1);
r0     = [X0,Y0,Z0];
Theta0 = zeros(nSeg0,1);
Vx0    = zeros(nSeg0,1);
Vy0    = zeros(nSeg0,1);
Vz0    = zeros(nSeg0,1);
dr0    = [Vx0,Vy0,Vz0];
w0     = zeros(nSeg0,1);
y0     = [r0,Theta0,dr0,w0];

%%% Initialization %%%%
Mld.dU = dU;
Mld.order = 4; % spline order
Mld.nSplineSamples = 101;
Mld.nControlPoints = 9;
Mld=setModel(Mld,sectionType,R1,R2,E,Bt,V,rho,FApos,FAneg,FBpos,FBneg,du,L,m,p,nd,nC); 
[Yinit, U ,Mld]= initialization(Mld,y0);
n_knot = length(U);
d = length(y0(1,:));         % number of components of each point control point P of B-Spline

% calculate the constrain matrix
% for reducing the computation time, this can be computed a priori
% ================================================================
Larray = cell (nC,Mld.nControlPoints);
Lx     = zeros(nC,Mld.nControlPoints);
Ly     = zeros(nC,Mld.nControlPoints);
Lz     = zeros(nC,Mld.nControlPoints);
Ltheta = zeros(nC,Mld.nControlPoints);
% constraints on two extremities/initial and final positions  (u(1),u(last))
% constraints will be posed on first and last control points (see the matrix Lg)
for cel=1:Mld.nControlPoints
    constraint=positionConstraints(Mld,1); % relativePositionConstraints(Mld,1,2);
    Larray{1,cel} = constraint{cel}; 
    constraint=positionConstraints(Mld,Mld.nSplineSamples); % relativePositionConstraints(Mld,Mld.nSplineSamples-1,Mld.nSplineSamples);
    % {(q(u0),1),(q(u1),-1)}
    % L = barycentricConstraints(Mld,a,location)
    Larray{2,cel} = constraint{cel};
end
for c=1:nC
    for n=1:length(Larray)
        Ldiag = diag(Larray{c,n});
        Lx(c,n)=Ldiag(1);
        Ly(c,n)=Ldiag(2);
        Lz(c,n)=Ldiag(3);
        Ltheta(c,n)=Ldiag(4);  
    end
end
Lx2 = Lx;
Lx = [];
Lx = Lx2(1,:);
% Mld.Lg=[Lx zeros(size(Lx,1),3*size(Lx,2));
%     zeros(size(Ly,1),size(Ly,2)) Ly zeros(size(Ly,1),2*size(Ly,2));
%     zeros(size(Lz,1),2*size(Lz,2)) Lz zeros(size(Lz,1),size(Lz,2));
%     zeros(size(Ltheta,1),3*size(Ltheta,2)) Ltheta];
Mld.Lg=[zeros(size(Lz,1),2*size(Lz,2)) Lz zeros(size(Lz,1),size(Lz,2))];
Mld.Fg = Mld.Mg*[zeros(1,Mld.nControlPoints) zeros(1,Mld.nControlPoints) -9.81*ones(1,Mld.nControlPoints) zeros(1,Mld.nControlPoints)]';
%     L = cell2mat(Larray);
%     Lx=L(1,:);
%     Ly=L(2,:);
%     Lz=L(3,:);
%     Ltheta=L(4,:);

% the following matrices are pre-computed to reduce the computation time. 
%Mld.ML = pinv(Mld.Mg)*Mld.Lg';
Mld.ML = Mld.pinvMg*Mld.Lg';
Mld.LML = Mld.Lg*Mld.ML;
Mld.LML_inv = pinv(Mld.LML);

    
% initialization of applied force
force_v = zeros(Mld.nControlPoints,3);           
torque  = zeros(Mld.nControlPoints,1);
% %% the force is applied in the middle of the wire
% force_v(floor(Mld.nSplineSamples/2),1) = 5000;   
% force_v(floor(Mld.nSplineSamples/2),2) = 5000;
% force_v(floor(Mld.nSplineSamples/2),3) = 5000;
% torque (floor(Mld.nSplineSamples/2),1) = 0;

% plots
figure
plot(Mld.ui,Mld.b)
xlim([min(U) max(U)])

figure (2)
hold on
grid on

%xlim([0 pf(3)])
xlabel('x')
ylabel('y')
zlabel('z')
ylim([-0.1 0.1])
zlim([-2 0])
view(144,18);

%patch([0 pf(3) pf(3) 0],[1 1 -1 -1],[floor_level floor_level floor_level floor_level],'r')
%plot3(X0,Y0,Z0,'mo', 'MarkerFaceColor','m')
LW1 = 1;
LW = 1.5;
P=Yinit(:,1:3)';     % control points  
Pc = plot3(P(1,:),P(2,:),P(3,:),'x');
set(Pc,'LineWidth',LW1);
%Pc = plot3(P(1,:),P(2,:),P(3,:),'--');
%set(Pc,'LineWidth',LW1);
%Pl=plot3(Mld.r0(:,1),Mld.r0(:,2),Mld.r0(:,3));
%set(Pl,'LineWidth',LW);

plot(P(1,:),P(2,:),'m-.')
plot(P(1,:),P(2,:),'mo')

pos_offset = -0.11:-0.01:-0.2;

force_scale = 0:0.05:1;
Tsim        = min(Tm):max(Tm)/length(force_scale):max(Tm);

