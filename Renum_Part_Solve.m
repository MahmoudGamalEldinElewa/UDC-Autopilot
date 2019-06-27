function [Dj,AR] = Renum_Part_Solve(Sj,Re,ndpos,Nn,Ac,Dj)
%% Renumbering and Partitioning
Sjre(Re,Re)=Sj;
S=Sjre(1:ndpos,1:ndpos);
Sdr=Sjre(1:ndpos,ndpos+1:2*Nn);
Srd=transpose(Sdr);
Srr=Sjre(ndpos+1:2*Nn,ndpos+1:2*Nn);
Acre(Re,1)=Ac;
Ad=Acre(1:ndpos);
Arl=-Acre(ndpos+1:2*Nn);
Djre(Re,1)=Dj;
Dr=Djre(ndpos+1:2*Nn);
%% Solution
D=S\(Ad-Sdr*Dr);
Djre=[D;Dr];
Dj=Djre(Re)
Ard=Srd*D+Srr*Dr;
AR=Arl+Ard

end



