function [] = plot_deformation(Connection,Dj,Nodes_Coordinates,Nm)
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
end

