function [mass,COM,Off]=load_est(Ft,Rt)
%LOAD_EST Estimates F/T sensor load (mass and COM)
%
%  Usage:
%    [mass,COM,Off]=load_est(Ft,Rt)
%
%  Input:
%    FT  Force/torque measurements (n x 6) or (6 x n)
%    Rt  sensor orientation matrix (3 x 3 x n) or quaternions (n x 4)
%
%  Output:
%    mass load mass
%    COM  center of mass (3 x 1)
%    Off  sensor offset
%

% Copyright (c) 2017 by IJS Leon Zlajpah
%

[n,mass]=size(Ft);
if n==6
    F=Ft(1:3,:);
    M=Ft(4:6,:);
    n=mass;
else
    F=Ft(:,1:3)';
    M=Ft(:,4:6)';
end
if ndims(Rt)==3
    validateattributes(Rt,{'numeric'},{'size',[3 3 n]},'load_est','Rt');
    R=Rt;
else
    R=zeros(3,3,n);
    if size(Rt,1)==4
        Rt=Rt';
    end
    for i=1:n
        R(:,:,i)=q2r(Rt(i,:));
    end
end
%% Estimate mass
%
% F = m*R'*g = -9.81*m*R'*[0 0 1]'= -9.81*m*R(3,:)'
%
% Fs = F + Foff
%

A=squeeze(R(3,:,:));
AI=[A(:) repmat(eye(3),[n 1])];
par=pinv(AI)*F(:);
mass=-par(1)/9.81;
Foff=par(2:4);


%% Estimate COM
%
% M = F x COM = -9.81*m*S(R(3,:)')*COM
%
% Ms = M + Moff
%

Fg=bsxfun(@minus,F',Foff')';
B=zeros(3*n,3);
for i=0:n-1
    B(3*i+1:3*i+3,:)=-v2s(Fg(:,i+1));
end
BI=[B repmat(eye(3),[n 1])];
par=pinv(BI)*M(:);
COM=par(1:3);
Moff=par(4:6);

Off=[Foff;Moff];

