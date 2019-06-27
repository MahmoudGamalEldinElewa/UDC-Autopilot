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
    Em=E(M_Info(i,1));
    Ame=Area(M_Info(i,2));
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

%% Get Fixed End Actions, Equivalent loads,Combined loads and  Renumbering DOF and Get number of possible DOF

[Ac,AE,Re,ndpos,Aml] = dewany(Nn,Nm,M_Load,N_Load,Connection,L,RL,RT);

%% Renumbering, Partitioning, Solving

[Dj,AR] = Renum_Part_Solve(Sj,Re,ndpos,Nn,Ac,Dj);

%% Plotting Deformation
DSF=100;
figure
hold all
for i=1:Nm
    N1=Connection(i,1);
    N2=Connection(i,2);
    Defx=Dj([2*N1-1,2*N2-1],1);
    Defy=Dj([2*N1,2*N2],1);
    Xvec=Nodes_Coordinates([N1 N2],1);
    Yvec=Nodes_Coordinates([N1 N2],2);
    plot(Xvec,Yvec,'--')
    plot(Xvec+DSF*Defx,Yvec+DSF*Defy,'-')
    title('Deformed Shape')
end
%% Plotting Diagrams
xbar1=[0:0.1:0.5]';
xbar2=xbar1+0.5;
Load=zeros(Nm,5);
id=M_Load(:,1);
Load(id,:)=M_Load(:,2:6);
for i=1:Nm
    N1=Connection(i,1);
    N2=Connection(i,2);
    Dof=[2*N1-1,2*N1,2*N2-1,2*N2];
    Dno=Dj(Dof,1);
    Lm=L(i);
    Am(i,:)=Aml(i,:)+transpose(RT(:,:,i)*Smd(:,:,i)*Dno);
    BM1=-Am(i,2)*xbar1*Lm-0.5*Load(i,5)*xbar1.^2*Lm^2;
    BM2=-Am(i,2)*xbar2*Lm-0.5*Load(i,5)*xbar2.^2*Lm^2-Load(i,3)*(xbar2-0.5)*Lm+Load(i,1);
    SF1=-Am(i,2)-Load(i,5)*xbar1*Lm;
    SF2=-Am(i,2)-Load(i,5)*xbar2*Lm-Load(i,3);
    TF1=-Am(i,1)-Load(i,4)*xbar1*Lm;
    TF2=-Am(i,1)-Load(i,4)*xbar2*Lm-Load(i,2);
    figure
    subplot(3,1,1)
    plot(xbar1,TF1,xbar2,TF2)
    xlabel('x_m')
    title(['Thrust Diagram of member ',num2str(i)])
    subplot(3,1,2)
    plot(xbar1,SF1,xbar2,SF2)
    xlabel('x_m')
    title(['Shear Diagram of member ',num2str(i)])
    subplot(3,1,3)
    plot(xbar1,BM1,xbar2,BM2)
    xlabel('x_m')
    title(['Bending Diagram of member ',num2str(i)])
end
Am
