clc
clearvars
close all
%% Inputs
P_1= 1e3; P_2= 2e3;
E=[7.3084e10];
Area=[2.38761e-4];
I=[4.32157e-8];
M_Info=[1,1;1,1;1,1;1,1;1,1;1,1;1,1;1,1;1,1;1,1;1,1;1,1;1,1;];
Nodes_Coordinates=[0,0;3,0;6,0;9,0;0,4;3,4;6,4;9,4];
Connection=[1,2;2,3;3,4;5,6;6,7;7,8;1,5;2,6;3,7;4,8;1,6;2,7;4,7];
Constr=[0,0;NaN,NaN;NaN,NaN;NaN,0;NaN,NaN;NaN,NaN;NaN,NaN;NaN,NaN];
N_Load=[5,P_1,-P_1;6,0,-P_1;7,0,-P_2;8,0,-P_1];
M_Load=[1,0,0,0,0,0];
%% Get Dj, RL, Local and Global Stiffness Matrices

[Nm,Nn,Dj,RL,RT,L,Sj,Smd] = Dj_RL_StiffnessMat(M_Info,Nodes_Coordinates,Constr,Connection,E,Area);
%% Get Fixed End Actions, Equivalent loads,Combined loads and  Renumbering DOF and Get number of possible DOF

[Ac,AE,Re,ndpos,Aml] = Aml_AE_Ac_Renum(Nn,Nm,M_Load,N_Load,Connection,L,RL,RT);

%% Renumbering, Partitioning, Solving

[Dj,AR] = Renum_Part_Solve(Sj,Re,ndpos,Nn,Ac,Dj);

%% Plotting Deformation
plot_deformation(Connection,Dj,Nodes_Coordinates,Nm)

%% Plotting Diagrams
Am=plot_diagrams(Connection,Dj,Nm,M_Load,Aml,RT,Smd,L);
Am
