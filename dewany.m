function [Ac,AE,Re,ndpos,Aml] = Aml_AE_Ac_Renum(Nn,Nm,M_Load,N_Load,Connection,L,RL,RT)
%% Get Fixed End Actions and Equivalent loads
AE=zeros(2*Nn,1);
Aml=zeros(Nm,4);
Amlb=Aml;
for i=1:size(M_Load,1)
   id=M_Load(i,1);
   Mm=M_Load(i,2);
   Pxm=M_Load(i,3);
   Pym=M_Load(i,4);
   wxm=M_Load(i,5);
   wym=M_Load(i,6);
   N1=Connection(id,1);
   N2=Connection(id,2);
   Lm=L(id);
   Dof=[2*N1-1,2*N1,2*N2-1,2*N2];
   
   Aml(id,1)=-Pxm/2-wxm*Lm/2;
   Aml(id,2)=Mm/Lm-Pym/2-wym*Lm/2;
   Aml(id,3)=Aml(id,1);
   Aml(id,4)=-Mm/Lm-Pym/2-wym*Lm/2;
   Amlb(id,:)=transpose(RT(:,:,id))*transpose(Aml(id,:));
   AE(Dof,1)=AE(Dof,1)-transpose(Amlb(id,:));    
end
%% Get Combined loads
A=zeros(2*Nn,1);
for i=1:size(N_Load,1)
    id=N_Load(i,1);
    Dof=(2*id-1):2*id;
    A(Dof,1)=N_Load(i,2:3);
    Ac=A+AE;
end
%% Renumbering DOF and Get number of possible DOF
ind=[find(RL==0);find(RL==1)];
Re(ind,1)=(1:2*Nn)';
ndpos=length(find(RL==0));
 
