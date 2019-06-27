function [Nm,Nn,Dj,RL,RT,L,Sj,Smd] = Dj_RL_StiffnessMat(M_Info,Nodes_Coordinates,Constr,Connection,E,Area)
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
end

