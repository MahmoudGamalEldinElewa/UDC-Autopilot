function [Am] = plot_diagrams(Connection,Dj,Nm,M_Load,Aml,RT,Smd,L)
%Plotting diagreams function
%   Arguments:
%   Connection: The vector containing the end points of the members 
%   Dj: Deformation vector
%   Nm: Number of Members
%   M_Load: the member loads
%   Aml: Fixed end actions 
%   RT: Rotation Matricies of the memebers
%   Smd: members structure stiffness matrices
%   L: Lenghthes of Members


%   Returns:
%   Am: Member end actions


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
end

