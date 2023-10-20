function [p,R,J]=kinmodel_talos_arm(q,tcp)
%KINMODEL_TALOS_ARM Kinematic model for TALOS arm uring URDF
% 
% Usage: 
%           [p,R,J]=kinmodel_talos_arm(q,tcp)
%           [p,R,J]=kinmodel_talos_arm(q)
% 
% Input: 
%           q   joint position (nj x 1) 
%           tcp tool center point 
% 
% Output:    
%           p   task position [x y z] (3 x 1) 
%           R   rotational matrix (3 x 3) 
%           J   Jacobian matrix (6 x nj) 
% 
% 

% Copyright (c) 2020 by IJS Leon Zlajpah 
% 

c1=cos(q(1));
s1=sin(q(1));
c2=cos(q(2));
s2=sin(q(2));
c3=cos(q(3));
s3=sin(q(3));
c4=cos(q(4));
s4=sin(q(4));
c5=cos(q(5));
s5=sin(q(5));
c6=cos(q(6));
s6=sin(q(6));
c7=cos(q(7));
s7=sin(q(7));


p1=0.157500;
p2=0.232000;
p3=0.004930;
p4=0.136500;
p5=0.046730;
p6=0.020000;
p7=-0.273000;
p8=-0.020000;
p9=-0.264300;

p=[...
    p6*(c1*c3 - c2*s1*s3) + p3*c1 - p4*s1 + p8*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) + p9*(s4*(c1*c3 - c2*s1*s3) + c4*s1*s2) + p7*s1*s2;...
    p1 + p6*(c3*s1 + c1*c2*s3) + p4*c1 + p3*s1 + p8*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) + p9*(s4*(c3*s1 + c1*c2*s3) - c1*c4*s2) - p7*c1*s2;...
    p2 + p5 - p8*(c2*s4 - c4*s2*s3) + p9*(c2*c4 + s2*s3*s4) + p7*c2 + p6*s2*s3;...
  ];

R=zeros(3,3);
R(1,1)=c7*(c5*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) - s5*(c1*s3 + c2*c3*s1)) - s7*(c6*(s4*(c1*c3 - c2*s1*s3) + c4*s1*s2) + s6*(s5*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) + c5*(c1*s3 + c2*c3*s1)));
R(1,2)=s6*(s4*(c1*c3 - c2*s1*s3) + c4*s1*s2) - c6*(s5*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) + c5*(c1*s3 + c2*c3*s1));
R(1,3)=c7*(c6*(s4*(c1*c3 - c2*s1*s3) + c4*s1*s2) + s6*(s5*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) + c5*(c1*s3 + c2*c3*s1))) + s7*(c5*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) - s5*(c1*s3 + c2*c3*s1));
R(2,1)=c7*(c5*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) - s5*(s1*s3 - c1*c2*c3)) - s7*(c6*(s4*(c3*s1 + c1*c2*s3) - c1*c4*s2) + s6*(s5*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) + c5*(s1*s3 - c1*c2*c3)));
R(2,2)=s6*(s4*(c3*s1 + c1*c2*s3) - c1*c4*s2) - c6*(s5*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) + c5*(s1*s3 - c1*c2*c3));
R(2,3)=c7*(c6*(s4*(c3*s1 + c1*c2*s3) - c1*c4*s2) + s6*(s5*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) + c5*(s1*s3 - c1*c2*c3))) + s7*(c5*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) - s5*(s1*s3 - c1*c2*c3));
R(3,1)=s7*(s6*(s5*(c2*s4 - c4*s2*s3) + c3*c5*s2) - c6*(c2*c4 + s2*s3*s4)) - c7*(c5*(c2*s4 - c4*s2*s3) - c3*s2*s5);
R(3,2)=c6*(s5*(c2*s4 - c4*s2*s3) + c3*c5*s2) + s6*(c2*c4 + s2*s3*s4);
R(3,3)=- s7*(c5*(c2*s4 - c4*s2*s3) - c3*s2*s5) - c7*(s6*(s5*(c2*s4 - c4*s2*s3) + c3*c5*s2) - c6*(c2*c4 + s2*s3*s4));

Jp=zeros(3,7);
Jp(1,1)=p7*c1*s2 - p4*c1 - p3*s1 - p8*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) - p9*(s4*(c3*s1 + c1*c2*s3) - c1*c4*s2) - p6*(c3*s1 + c1*c2*s3);
Jp(1,2)=p9*(c2*c4*s1 + s1*s2*s3*s4) - p8*(c2*s1*s4 - c4*s1*s2*s3) + p7*c2*s1 + p6*s1*s2*s3;
Jp(1,3)=- p6*(c1*s3 + c2*c3*s1) - p8*c4*(c1*s3 + c2*c3*s1) - p9*s4*(c1*s3 + c2*c3*s1);
Jp(1,4)=p9*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) - p8*(s4*(c1*c3 - c2*s1*s3) + c4*s1*s2);
Jp(1,5)=0;
Jp(1,6)=0;
Jp(1,7)=0;
Jp(2,1)=p6*(c1*c3 - c2*s1*s3) + p3*c1 - p4*s1 + p8*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) + p9*(s4*(c1*c3 - c2*s1*s3) + c4*s1*s2) + p7*s1*s2;
Jp(2,2)=p8*(c1*c2*s4 - c1*c4*s2*s3) - p9*(c1*c2*c4 + c1*s2*s3*s4) - p7*c1*c2 - p6*c1*s2*s3;
Jp(2,3)=- p6*(s1*s3 - c1*c2*c3) - p8*c4*(s1*s3 - c1*c2*c3) - p9*s4*(s1*s3 - c1*c2*c3);
Jp(2,4)=p9*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) - p8*(s4*(c3*s1 + c1*c2*s3) - c1*c4*s2);
Jp(2,5)=0;
Jp(2,6)=0;
Jp(2,7)=0;
Jp(3,1)=0;
Jp(3,2)=p8*(s2*s4 + c2*c4*s3) - p9*(c4*s2 - c2*s3*s4) - p7*s2 + p6*c2*s3;
Jp(3,3)=p6*c3*s2 + p8*c3*c4*s2 + p9*c3*s2*s4;
Jp(3,4)=- p8*(c2*c4 + s2*s3*s4) - p9*(c2*s4 - c4*s2*s3);
Jp(3,5)=0;
Jp(3,6)=0;
Jp(3,7)=0;

Jr=zeros(3,7);
Jr(1,1)=0;
Jr(1,2)=c1;
Jr(1,3)=s1*s2;
Jr(1,4)=- c1*s3 - c2*c3*s1;
Jr(1,5)=s4*(c1*c3 - c2*s1*s3) + c4*s1*s2;
Jr(1,6)=c5*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) - s5*(c1*s3 + c2*c3*s1);
Jr(1,7)=s6*(s4*(c1*c3 - c2*s1*s3) + c4*s1*s2) - c6*(s5*(c4*(c1*c3 - c2*s1*s3) - s1*s2*s4) + c5*(c1*s3 + c2*c3*s1));
Jr(2,1)=0;
Jr(2,2)=s1;
Jr(2,3)=-c1*s2;
Jr(2,4)=c1*c2*c3 - s1*s3;
Jr(2,5)=s4*(c3*s1 + c1*c2*s3) - c1*c4*s2;
Jr(2,6)=c5*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) - s5*(s1*s3 - c1*c2*c3);
Jr(2,7)=s6*(s4*(c3*s1 + c1*c2*s3) - c1*c4*s2) - c6*(s5*(c4*(c3*s1 + c1*c2*s3) + c1*s2*s4) + c5*(s1*s3 - c1*c2*c3));
Jr(3,1)=1;
Jr(3,2)=0;
Jr(3,3)=c2;
Jr(3,4)=c3*s2;
Jr(3,5)=c2*c4 + s2*s3*s4;
Jr(3,6)=c3*s2*s5 - c5*(c2*s4 - c4*s2*s3);
Jr(3,7)=c6*(s5*(c2*s4 - c4*s2*s3) + c3*c5*s2) + s6*(c2*c4 + s2*s3*s4);

if nargin==2
    if isequal(size(tcp),[4 4])
        p_tcp=tcp(1:3,4);
        R_tcp=tcp(1:3,1:3);
    elseif isequal(size(tcp),[1 3])
        p_tcp=tcp(1:3)';
        R_tcp=eye(3);
    elseif isequal(size(tcp),[3 1])
        p_tcp=tcp(1:3);
        R_tcp=eye(3);
    elseif isequal(size(tcp),[1 7])
        p_tcp=tcp(1:3)';
        R_tcp=q2r(tcp(4:7));
    elseif isequal(size(tcp),[7 1])
        p_tcp=tcp(1:3);
        R_tcp=q2r(tcp(4:7));
    elseif isequal(size(tcp),[1 6])
        p_tcp=tcp(1:3)';
        R_tcp=rpy2r(tcp(4:6));
    elseif isequal(size(tcp),[6 1])
        p_tcp=tcp(1:3);
        R_tcp=rpy2r(tcp(4:6));
    else
        error('kinmodel: wrong tcp form')
    end

    p=p+R*p_tcp;
    Jp=Jp+v2s(R*p_tcp)'*Jr;
    R=R*R_tcp;
end

J=[Jp;Jr];


