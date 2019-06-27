function [Dj,AR] = Renum_Part_Solve(Sj,Re,ndpos,Nn,Ac,Dj)

%Renumbering function
%
%   Arguments:
%   Sj: Total structure stiffness matrices in member axis
%   Re: Renumbering order
%   ndpos: The position of restrained DOF
%   Nn: Number of nodes
%   Ac: Combined loads vector
%   Dj: Deformation vector



%   Returns:
%   Dj: Deformation vector
%   AR: Restrained end actions

%% Renumbering and Partitioning
Sjre(Re,Re)=Sj;                         % Renumbered Joint Stiffness Matrix
S=Sjre(1:ndpos,1:ndpos);                % Get S
Sdr=Sjre(1:ndpos,ndpos+1:2*Nn);         % Get Sdr
Srd=transpose(Sdr);                     % Get Srd
Srr=Sjre(ndpos+1:2*Nn,ndpos+1:2*Nn);    % Get Srr
Acre(Re,1)=Ac;                          % Renumber Ac
Ad=Acre(1:ndpos);                       % get Ad
Arl=-Acre(ndpos+1:2*Nn);                % get Arl
Djre(Re,1)=Dj;                          % Renumber displacements vector
Dr=Djre(ndpos+1:2*Nn);                  % restrained displacements vector
%% Solution
D=S\(Ad-Sdr*Dr);                        % Get possible displacments
Djre=[D;Dr];                            % Structure total displacements renumbered
Dj=Djre(Re)                             % Structure total displacements arbitrary
Ard=Srd*D+Srr*Dr;                       % Get Ard
AR=Arl+Ard                              % Get total structure reactions

end



