%Add path to custom libraries
addpath("mr\")
addpath("Helper_functions\")

L1 = 350e-3;
L2 = 830e-3;
L3 = 1160e-3;
L4 = 250e-3;
L5 = 1492.18e-3;
L6 = 210e-3; % Last length plus End-effector/hook
L7 = 567e-3; %winch sheeve extension 567e-3


%Zero configuration matrix
M = [1 0 0 L1+L5+L6+L7;
     0 1 0 0;
     0 0 1 L2+L3+L4;
     0 0 0 1];

%thetaList0 = [0 0 -1.57 0 0];

%Defining Screw axis

%omega direction of screw axis
omega1 = [0 0 -1]';
omega2 = [0 1 0]';
omega3 = [0 -1 0]';
omega4 = [0 1 0]';
omega5 = [0 0 1]';

%q some arbitrary point along the screw axis
q1 = [0 0 0]';
q2 = [L1 0 L2]';
q3 = [L1 0 L2+L3]';
q4 = [L1+L5+L6+L7 0 L2+L3+L4]';
q5 = [L1+L5+L6+L7 0 L2+L3+L4]';

omega_mat = [omega1 omega2 omega3 omega4 omega5];

%The negative kinematic moment of the screw axis (v) at an arbitrary point (q)
% -omega X q
v_mat = [-cross(omega1,q1) -cross(omega2,q2) -cross(omega3,q3)...
         -cross(omega4,q4) -cross(omega5,q5)];


SList = [omega_mat; v_mat];

%For body FK IK and jacobian
M_B = TransInv(M);
BList = Adjoint(M_B) * SList;

pars.controlMode = 2;
pars.M         = M;
pars.SList     = SList;
pars.x_coordinate_traj = 0;
pars.y_coordinate_traj = 0;
pars.z_coordinate_traj = 0;
pars.new_command_traj = 0;
pars.Traj_method = 1;
pars.v_max = 0;
pars.Operation_Mode = 2;
pars.Manual_State_Switch = 0;
pars.Winch_PosRef = 10;
pars.Winch_NewCmd = 0;
pars.Winch_EnableHeaveComp = 0;
pars.BList = BList;
pars.State_Mode_Selector = 2;
%pars.AutoMode_flag = 0;