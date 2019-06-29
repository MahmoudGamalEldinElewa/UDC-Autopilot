clc
clearvars
close all
%% Inputs
P_1= 1e3; P_2= 2e3;
E=[7.3084e10];
Area=[2.38761e-4];
I=[4.32157e-8];
%%edited
M_Info=[Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1);Area(1),E(1)];
Nodes_Coordinates=[0,0;3,0;6,0;9,0;0,4;3,4;6,4;9,4];
Connection=[1,2;2,3;3,4;5,6;6,7;7,8;1,5;2,6;3,7;4,8;1,6;2,7;4,7];
Constr=[0,0;NaN,NaN;NaN,NaN;NaN,0;NaN,NaN;NaN,NaN;NaN,NaN;NaN,NaN];
N_Load=[5,P_1,-P_1;6,0,-P_1;7,0,-P_2;8,0,-P_1];
M_Load=[1,0,0,0,0,0];
%% Get Dj and RL
Nm=size(M_Info,1);
Nn=size(Nodes_Coordinates,1);
Dj(:,1)=reshape(Constr',[2*Nn,1]);
RL=1-isnan(Dj);
%% Get Local and Global Stiffness Matrices
Sj=sparse(2*Nn,2*Nn);
for i=1:Nm
    N1=Connection(i,1);
    N2=Connection(i,2);
    dx=Nodes_Coordinates(N2,1)-Nodes_Coordinates(N1,1);
    dy=Nodes_Coordinates(N2,2)-Nodes_Coordinates(N1,2);
    Lm=sqrt(dx^2+dy^2);
    Cx=dx/Lm;
    Cy=dy/Lm;
    %%edited
    Em=M_Info(i,2);
    Ame=M_Info(i,1);
    Dof=[2*N1-1,2*N1,2*N2-1,2*N2];    
    Smd(:,:,i)=Em*Ame/Lm*[Cx^2 Cx*Cy -Cx^2 -Cx*Cy;
                          Cx*Cy Cy^2 -Cx*Cy -Cy^2;
                          -Cx^2 -Cx*Cy Cx^2 Cx*Cy;
                          -Cx*Cy -Cy^2 Cx*Cy Cy^2];
    Sj(Dof,Dof)=Sj(Dof,Dof)+Smd(:,:,i);
    RT(:,:,i)= [Cx Cy 0 0;
               -Cy Cx 0 0;
                0 0 Cx Cy;
                0 0 -Cy Cx];
    L(i)=Lm;
end