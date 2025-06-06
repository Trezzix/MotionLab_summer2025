clc;
clear all;
close all;

%Add path to custom libraries
addpath("mr\")
addpath("Helper_functions\")


L1 = 350e-3;
H2 = 830e-3;
H3 = 1160e-3;
H4 = 250e-3;
L5 = 1492.18e-3;
L6 = 210e-3; % Last length plus End-effector/hook
L7 = 567e-3; %winch sheeve extension

%Links of the parrallel mechanism
L8 = 320e-3;
L9 = 1160e-3;
L10 = 320e-3;


%Zero configuration matrix
M = [1 0 0 L1+L5+L6+L7;
     0 1 0 0;
     0 0 1 H2+H3+H4;
     0 0 0 1];
% M = [1 0 0 L1-H4;
%      0 1 0 0;
%      0 0 1 H2+H3+L5+L6+L7;
%      0 0 0 1];

%Defining Screw axis

%omega direction of screw axis
omega1 = [0 0 -1]';
omega2 = [0 1 0]';
omega3 = [0 -1 0]';
%omega4 = [0 1 0]';
%omega5 = [0 0 1]';

%q some arbitrary point along the screw axis
q1 = [0 0 0]';
q2 = [L1 0 H2]';
q3 = [L1 0 H2+H3]';
%q4 = [L1+L5+L6+L7 0 H2+H3+H4]';
%q5 = [L1+L5+L6+L7 0 H2+H3+H4]';

omega_mat = [omega1 omega2 omega3]; %omega4 omega5];

%The negative kinematic moment of the screw axis (v) at an arbitrary point (q)
% -omega X q
v_mat = [-cross(omega1,q1) -cross(omega2,q2) -cross(omega3,q3)];%...
         %-cross(omega4,q4) -cross(omega5,q5)];


SList = [omega_mat; v_mat];

thetaList0 = [0 0 0 0 0];

%For body FK IK and jacobian
M_B = TransInv(M);
BList = Adjoint(M_B) * SList;

%T = FKinSpace(M, SList, thetaList0)


%[thetaList, success] = IKinSpace(SList, M, T, thetaList0, 0.00001, 0.0000001)

%Simulink parameters

%timestep
Ts = 0.001;
%thetaList0 = [0 0 -pi/2 0 0 0];

%From world frame to platform frame
w_s_quat = [6.0198987883e-05, 0.705931944115, 0.708279565403954, 0.000379322138596];

R_w_p = quat2rotm(w_s_quat);
%tr_w_p = [-2.969661622702265, 1.976311036084776, 2.662845015592614];
tr_w_p = [-2.969661622702265, 1.976311036084776, 3.312845015592614];

%platform frame to comau
p_c_quat = [0.002461288106714, -0.501408337431486, -0.865206982556575, 0.000706081011787];

R_p_c = quat2rotm(p_c_quat);

tr_p_c = [-1.082024099747949, 1.536041691521612, -0.3];
%tr_p_c = [-1.082024099747949, 1.536041691521612, -1.024461448780775];

%% SMART_5NJ_110_3_0_DataFile
%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiDataRobot.RigidTransform(16).translation = [0.0 0.0 0.0];
smiDataRobot.RigidTransform(16).angle = 0.0;
smiDataRobot.RigidTransform(16).axis = [0.0 0.0 0.0];
smiDataRobot.RigidTransform(16).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(1).translation = [0 331 0];  % mm
smiDataRobot.RigidTransform(1).angle = 0;  % rad
smiDataRobot.RigidTransform(1).axis = [0 0 0];
smiDataRobot.RigidTransform(1).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p02.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(2).translation = [-450 2240 0];  % mm
smiDataRobot.RigidTransform(2).angle = 0;  % rad
smiDataRobot.RigidTransform(2).axis = [0 0 0];
smiDataRobot.RigidTransform(2).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p05.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(3).translation = [-350 830 294.75];  % mm
smiDataRobot.RigidTransform(3).angle = 3.1415926535897931;  % rad
smiDataRobot.RigidTransform(3).axis = [1 0 0];
smiDataRobot.RigidTransform(3).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_CR82350080_SM.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(4).translation = [-1842.1799999999998 2240 9.9999999999999998e-13];  % mm
smiDataRobot.RigidTransform(4).angle = 0;  % rad
smiDataRobot.RigidTransform(4).axis = [0 0 0];
smiDataRobot.RigidTransform(4).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p08.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(5).translation = [-1917.4300000000001 2240 -4.9999999999999997e-12];  % mm
smiDataRobot.RigidTransform(5).angle = 1.5707963267948977;  % rad
smiDataRobot.RigidTransform(5).axis = [-1.4999999999999981e-15 -1 -1.4999999999999981e-15];
smiDataRobot.RigidTransform(5).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_CR82350030_SM.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(6).translation = [0 0 0];  % mm
smiDataRobot.RigidTransform(6).angle = 0;  % rad
smiDataRobot.RigidTransform(6).axis = [0 0 0];
smiDataRobot.RigidTransform(6).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p01.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(7).translation = [-240.25 2190 169.72568626429799];  % mm
smiDataRobot.RigidTransform(7).angle = 2.0943951023931966;  % rad
smiDataRobot.RigidTransform(7).axis = [0.57735026918962562 -0.57735026918962606 -0.57735026918962562];
smiDataRobot.RigidTransform(7).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_CR82350050_SM.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(8).translation = [-350 830 135.5];  % mm
smiDataRobot.RigidTransform(8).angle = 0;  % rad
smiDataRobot.RigidTransform(8).axis = [0 0 0];
smiDataRobot.RigidTransform(8).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p03.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(9).translation = [-350 830 -350.25];  % mm
smiDataRobot.RigidTransform(9).angle = 3.1415926535897913;  % rad
smiDataRobot.RigidTransform(9).axis = [-0 -0 -1];
smiDataRobot.RigidTransform(9).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_CR82350070_SM.STEP-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(10).translation = [-31.217696610640999 802.11016232075008 -179.5];  % mm
smiDataRobot.RigidTransform(10).angle = 0;  % rad
smiDataRobot.RigidTransform(10).axis = [0 0 0];
smiDataRobot.RigidTransform(10).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p06.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(11).translation = [-350 830 -194.5];  % mm
smiDataRobot.RigidTransform(11).angle = 0.087266462599717279;  % rad
smiDataRobot.RigidTransform(11).axis = [0 -1.1451882774215491e-14 -1];
smiDataRobot.RigidTransform(11).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p04.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(12).translation = [-2007.179999999981 2240.0000000000132 -6.4999999999999995e-11];  % mm
smiDataRobot.RigidTransform(12).angle = 0;  % rad
smiDataRobot.RigidTransform(12).axis = [0 0 0];
smiDataRobot.RigidTransform(12).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p10.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(13).translation = [156.75059808498401 472.5 90.5];  % mm
smiDataRobot.RigidTransform(13).angle = 2.7734925708571119;  % rad
smiDataRobot.RigidTransform(13).axis = [0.1861567878973851 0.69474659060686583 -0.69474659060686583];
smiDataRobot.RigidTransform(13).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_CR82350070_SM.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(14).translation = [-1967.1799999999998 2239.9999999999964 -2.5000000000000001e-11];  % mm
smiDataRobot.RigidTransform(14).angle = 1.5707963267948954;  % rad
smiDataRobot.RigidTransform(14).axis = [-4.5000000000000005e-15 1 3.5000000000000001e-15];
smiDataRobot.RigidTransform(14).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p09.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(15).translation = [-495 2239.9999999999982 -2.5000000000000001e-11];  % mm
smiDataRobot.RigidTransform(15).angle = 2.0943951023932024;  % rad
smiDataRobot.RigidTransform(15).axis = [-0.57735026918962851 -0.57735026918962118 0.57735026918962751];
smiDataRobot.RigidTransform(15).ID = "AssemblyGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1:SMART-5 NJ-110-3.0_p07.STEP-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiDataRobot.RigidTransform(16).translation = [0 0 0];  % mm
smiDataRobot.RigidTransform(16).angle = 0;  % rad
smiDataRobot.RigidTransform(16).axis = [0 0 0];
smiDataRobot.RigidTransform(16).ID = "RootGround[SMART-5 NJ-110-3.0 (3D-STEP).STEP-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiDataRobot.Solid(14).mass = 0.0;
smiDataRobot.Solid(14).CoM = [0.0 0.0 0.0];
smiDataRobot.Solid(14).MoI = [0.0 0.0 0.0];
smiDataRobot.Solid(14).PoI = [0.0 0.0 0.0];
smiDataRobot.Solid(14).color = [0.0 0.0 0.0];
smiDataRobot.Solid(14).opacity = 0.0;
smiDataRobot.Solid(14).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(1).mass = 52.706031176315982;  % kg
smiDataRobot.Solid(1).CoM = [-242.66897813542667 296.14549865714821 12.560965025190244];  % mm
smiDataRobot.Solid(1).MoI = [4136639.3686790983 3647453.6565928138 4173251.5664203875];  % kg*mm^2
smiDataRobot.Solid(1).PoI = [21269.015167562859 -54887.133975236575 1321088.2058438642];  % kg*mm^2
smiDataRobot.Solid(1).color = [0.203 0.210 0.239];
smiDataRobot.Solid(1).opacity = 1;
smiDataRobot.Solid(1).ID = "SMART-5 NJ-110-3.0_p02.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(2).mass = 25.314195721014904;  % kg
smiDataRobot.Solid(2).CoM = [81.922886858874577 -33.833989402443976 14.448782406787638];  % mm
smiDataRobot.Solid(2).MoI = [783008.26269064494 548584.67016851983 525604.1322728931];  % kg*mm^2
smiDataRobot.Solid(2).PoI = [-112599.38170428496 21299.174138007162 72588.874515329284];  % kg*mm^2
smiDataRobot.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(2).opacity = 1;
smiDataRobot.Solid(2).ID = "SMART-5 NJ-110-3.0_p05.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(3).mass = 9.4493198216073964;  % kg
smiDataRobot.Solid(3).CoM = [0.00035605587580101627 0.59254727846081734 -131.34248514352112];  % mm
smiDataRobot.Solid(3).MoI = [96683.905101085853 96090.527287838268 47612.014811029614];  % kg*mm^2
smiDataRobot.Solid(3).PoI = [678.15178926669853 -1.9373731491872181 -0.37884372181229253];  % kg*mm^2
smiDataRobot.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(3).opacity = 1;
smiDataRobot.Solid(3).ID = "SMART-5 NJ-110-3.0_CR82350080_SM.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(4).mass = 27.907822986627043;  % kg
smiDataRobot.Solid(4).CoM = [176.91381036358237 3.8341896469272663 1.9754510358205102];  % mm
smiDataRobot.Solid(4).MoI = [482005.2410957645 820993.89522460627 552912.66986302414];  % kg*mm^2
smiDataRobot.Solid(4).PoI = [-5805.283149630598 -66627.605911827952 -6375.7492384737961];  % kg*mm^2
smiDataRobot.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(4).opacity = 1;
smiDataRobot.Solid(4).ID = "SMART-5 NJ-110-3.0_p08.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(5).mass = 1.8204370609575367;  % kg
smiDataRobot.Solid(5).CoM = [0.0011781120658499607 1.8532136422554566 -83.079832406507592];  % mm
smiDataRobot.Solid(5).MoI = [7475.3672830480818 7269.0527134527229 2858.8228291073215];  % kg*mm^2
smiDataRobot.Solid(5).PoI = [189.23833790092377 0.076539858692730889 -0.14274057644209939];  % kg*mm^2
smiDataRobot.Solid(5).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(5).opacity = 1;
smiDataRobot.Solid(5).ID = "SMART-5 NJ-110-3.0_CR82350030_SM.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(6).mass = 98.712536679746705;  % kg
smiDataRobot.Solid(6).CoM = [36.183943831325131 132.25114554111747 -0.030525648643372944];  % mm
smiDataRobot.Solid(6).MoI = [3268478.8711421788 6102276.4254572866 4269111.0143532446];  % kg*mm^2
smiDataRobot.Solid(6).PoI = [-35.583053939592197 631.47068636807955 -2524.4017230519235];  % kg*mm^2
smiDataRobot.Solid(6).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(6).opacity = 1;
smiDataRobot.Solid(6).ID = "SMART-5 NJ-110-3.0_p01.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(7).mass = 3.5666359355844257;  % kg
smiDataRobot.Solid(7).CoM = [1.2461000205248818 -0.0010192432622059655 -85.200466358919499];  % mm
smiDataRobot.Solid(7).MoI = [17145.157040564802 17509.336534066519 10295.619245635809];  % kg*mm^2
smiDataRobot.Solid(7).PoI = [-0.163954126982988 300.11359564551077 0.32104894325969818];  % kg*mm^2
smiDataRobot.Solid(7).color = [0.6392157 0.0 0.0];
smiDataRobot.Solid(7).opacity = 1;
smiDataRobot.Solid(7).ID = "SMART-5 NJ-110-3.0_CR82350050_SM.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(8).mass = 22.854890403146104;  % kg
smiDataRobot.Solid(8).CoM = [-23.928506665749911 523.99566525340538 -159.70044895169784];  % mm
smiDataRobot.Solid(8).MoI = [3177590.9155515293 341455.30737882649 3069690.399421786];  % kg*mm^2
smiDataRobot.Solid(8).PoI = [51188.540610659591 -6971.0396180961588 7197.3923801430437];  % kg*mm^2
smiDataRobot.Solid(8).color = [0.64 0.0 0.0];
smiDataRobot.Solid(8).opacity = 1;
smiDataRobot.Solid(8).ID = "SMART-5 NJ-110-3.0_p03.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(9).mass = 8.5814594224327365;  % kg
smiDataRobot.Solid(9).CoM = [0.00039190881468678792 0.65247306493897117 -117.33220651590989];  % mm
smiDataRobot.Solid(9).MoI = [76220.255867257161 75627.213475154422 43301.239657847611];  % kg*mm^2
smiDataRobot.Solid(9).PoI = [598.99194041261785 -1.5272912821901943 -0.37849880267797315];  % kg*mm^2
smiDataRobot.Solid(9).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(9).opacity = 1;
smiDataRobot.Solid(9).ID = "SMART-5 NJ-110-3.0_CR82350070_SM.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(10).mass = 7.5792666918117497;  % kg
smiDataRobot.Solid(10).CoM = [65.147048462687252 508.83792056545542 14.747257945702893];  % mm
smiDataRobot.Solid(10).MoI = [667373.64067244437 18083.172326137388 678945.36689657171];  % kg*mm^2
smiDataRobot.Solid(10).PoI = [-225.7042116055666 46.998810275833364 17500.803228922214];  % kg*mm^2
smiDataRobot.Solid(10).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(10).opacity = 1;
smiDataRobot.Solid(10).ID = "SMART-5 NJ-110-3.0_p06.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(11).mass = 4.3287759696132397;  % kg
smiDataRobot.Solid(11).CoM = [163.57070996069299 -9.8617817182830834 27.536485713357724];  % mm
smiDataRobot.Solid(11).MoI = [15456.864188549995 65374.86371961276 72810.661297813465];  % kg*mm^2
smiDataRobot.Solid(11).PoI = [-165.91415506329329 512.48565629209236 3817.7544015007338];  % kg*mm^2
smiDataRobot.Solid(11).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(11).opacity = 1;
smiDataRobot.Solid(11).ID = "SMART-5 NJ-110-3.0_p04.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(12).mass = 0.47985825240457192;  % kg
smiDataRobot.Solid(12).CoM = [-27.34791626851143 -1.6026209534547153 -5.1001180974803206e-05];  % mm
smiDataRobot.Solid(12).MoI = [1618.4776017805987 894.29456427734294 837.21364973354628];  % kg*mm^2
smiDataRobot.Solid(12).PoI = [-7.7780608542633322e-06 0.00054638186946613686 4.4070273941284572];  % kg*mm^2
smiDataRobot.Solid(12).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(12).opacity = 1;
smiDataRobot.Solid(12).ID = "SMART-5 NJ-110-3.0_p10.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(13).mass = 2.1916043056807371;  % kg
smiDataRobot.Solid(13).CoM = [-0.46659407311739648 -0.96652135243620041 26.711273550845025];  % mm
smiDataRobot.Solid(13).MoI = [11436.827713254688 13423.173203473376 8596.8063115297427];  % kg*mm^2
smiDataRobot.Solid(13).PoI = [153.22479160131755 -298.20565802941911 -10.064478096809076];  % kg*mm^2
smiDataRobot.Solid(13).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(13).opacity = 1;
smiDataRobot.Solid(13).ID = "SMART-5 NJ-110-3.0_p09.STEP*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiDataRobot.Solid(14).mass = 18.744700041112438;  % kg
smiDataRobot.Solid(14).CoM = [-0.005958152306662661 -0.089401100201608463 460.33855874646821];  % mm
smiDataRobot.Solid(14).MoI = [1632832.4693022415 1632553.6239231301 67074.935683703909];  % kg*mm^2
smiDataRobot.Solid(14).PoI = [-835.38260715793831 53.677126594132538 -8.4899136334609864];  % kg*mm^2
smiDataRobot.Solid(14).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiDataRobot.Solid(14).opacity = 1;
smiDataRobot.Solid(14).ID = "SMART-5 NJ-110-3.0_p07.STEP*:*Default";