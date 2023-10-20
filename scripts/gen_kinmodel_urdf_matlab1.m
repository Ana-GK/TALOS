function [nj]=gen_kinmodel_urdf_matlab(robot,description,urdf,initial_link,final_link,prefix)
%GEN_KINMODEL_URDF_MATLAB Generates kinematic model for robot from URDF
% and write the model to file
%
% Input:
%   robot           robot name or structure
%   description     robot description
%   urdf            URDF filename
%   initial_link    name of first link (optional)
%   final_link      name of last link (optional)
%   prefix          prefix for filename for generated model (optional)
%
%   name   structure: (alternative)
%           .name            robot name or structure
%           .description     robot description
%           .urdf            URDF filename
%           .initial_link    name of first link (optional)
%           .final_link      name of last link (optional)
%           .prefix          prefix for filename for generated model (optional)
%
% Output:
%       nj      number of active joints
%
%       kinematic model file: <prefix>kinmodel_<name>.m
%

% Copyright (c) 2021 by IJS Leon Zlajpah
%

if ~isstruct(robot)
    robot.name=robotname;
    robot.description=description;
    robot.urdf=urdf;
    robot.initial_link=initial_link;
    robot.final_link=final_link;
    if nargin==6
        robot.prefix=prefix;
    end
end

[~,~,ext]=fileparts(robot.urdf);
if isempty(ext)
    robot.urdf=[robot.urdf '.urdf'];
end

if ~isfield(robot,'prefix')
    robot.prefix='';
end
robot.m_name=[robot.prefix 'kinmodel_' robot.name];

robot.urdf=which(robot.urdf);
if isfile(robot.urdf)
    model=xml2struct(robot.urdf);
else
    error('File ''%s'' does not exists',robot.urdf)
end

kin.NJ=size(model.robot.joint,2);
if kin.NJ == 1
    model.robot.joint=num2cell(model.robot.joint);
end

kin.joint_typ={};

if isfield(robot,'initial_link')
    if ~isfield(robot,'final_link')
        robot.final_link='world';
    end
    idxs=strcmp(cellfun(@(joi) joi.child.Attributes.link, model.robot.joint, 'uni', false ), {robot.final_link});
    end_idx=find(idxs==1);
    if isempty(end_idx)
        error('Invalid last link name')
    end
    joints=[end_idx];
    start_idx=end_idx;
    while ~strcmp(model.robot.joint{start_idx}.parent.Attributes.link,robot.initial_link) && ~isempty(start_idx)
        idxs=strcmp(cellfun(@(joi) joi.child.Attributes.link,model.robot.joint,'uni',false),{model.robot.joint{start_idx}.parent.Attributes.link});
        start_idx=find(idxs==1);
        if isempty(start_idx)
            error('Invalid first link name')
        else
            joints=[start_idx joints];
        end
    end
else
    joints=1:kin.NJ;
end

%Generation loop
kin.nj=0;
a_sym=[];
A=[];
a_n=0;
p_sym=[];
P=[];
p_n=0;
for i=1:length(joints)
    ijoint=joints(i);
    %Joint type
    if strcmpi(model.robot.joint{ijoint}.Attributes.type,'revolute')|| strcmpi(model.robot.joint{ijoint}.Attributes.type,'continuous')
        kin.joint_rev(i)=1;
        if strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'1 0 0')
            kin.joint_typ{i}='Rx';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'-1 0 0')
            kin.joint_typ{i}='-Rx';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'0 1 0')
            kin.joint_typ{i}='Ry';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'0 -1 0')
            kin.joint_typ{i}='-Ry';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'0 0 1')
            kin.joint_typ{i}='Rz';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'0 0 -1')
            kin.joint_typ{i}='-Rz';
        end
        kin.nj=kin.nj+1;
        kin.joint_ind(i)=kin.nj;
    elseif strcmpi(model.robot.joint{ijoint}.Attributes.type,'prismatic')
        kin.joint_rev(i)=2;
        if strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'1 0 0')
            kin.joint_typ{i}='Px';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'-1 0 0')
            kin.joint_typ{i}='-Px';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'0 1 0')
            kin.joint_typ{i}='Py';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'0 -1 0')
            kin.joint_typ{i}='-Py';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'0 0 1')
            kin.joint_typ{i}='Pz';
        elseif strcmp(model.robot.joint{ijoint}.axis.Attributes.xyz,'0 0 -1')
            kin.joint_typ{i}='-Pz';
        end
        kin.nj=kin.nj+1;
        kin.joint_ind(i)=kin.nj;
    elseif strcmpi(model.robot.joint{ijoint}.Attributes.type,'fixed')
        kin.joint_rev(i)=0;
        kin.joint_typ{i}='F';
        kin.joint_ind(i)=0;
    end

    %Joint transformation
    angle=str2num(model.robot.joint{ijoint}.origin.Attributes.rpy);
    pos=str2num(model.robot.joint{ijoint}.origin.Attributes.xyz);
    ang_sym=sym(angle);
    pos_sym=sym(pos);
    for k=1:3
        if angle(k)~=0
            a=angle(k)/pi*2;
            if abs(a-round(a))<0.0001
                ang_sym(k)=sym(round(a)/2*sym(pi));
            else
                a_n=a_n+1;
                a_sym(a_n)=angle(k);
                ang_sym(k)=sym(['a',int2str(a_n)],'real');
            end
        end
        if pos(k)~=0
            p_n=p_n+1;
            p_sym(p_n)=pos(k);
            pos_sym(k)=sym(['p',int2str(p_n)],'real');
        end
    end
    kin.joint_T{i}=rp2t(rot_z(ang_sym(3))*rot_y(ang_sym(2))*rot_x(ang_sym(1)),pos_sym);
end

fprintf('All joints included in the model (%d): ',kin.NJ)
fprintf('%d ',joints)
fprintf('\n')
fprintf('Active joints included in the model: %d\n',kin.nj)


nj=kin.nj;
Q=sym('q',[nj,1],'real');

T=sym(eye(4));
TX=sym(zeros(4,4,nj+1));
JX=sym(zeros(3,1,nj+1));
TT=sym(zeros(4,4,nj));

for i=1:length(joints)
    if kin.joint_ind(i)>0
        if kin.joint_rev(i)==1 % revolute
            TX(:,:,kin.joint_ind(i))=T;
            TT(:,:,i)=kin.joint_T{i}*joint_transform(kin.joint_typ{i},Q(kin.joint_ind(i)));
            JX(:,kin.joint_ind(i))=T(1:3,1:3)*joint_axis(kin.joint_typ{i});
        else% prismatic
            TX(:,:,kin.joint_ind(i))=T;
            TT(:,:,i)=kin.joint_T{i}*joint_transform(kin.joint_typ{i},Q(kin.joint_ind(i)));
            JX(:,kin.joint_ind(i))=zeros(3,1);
        end
    else
        TT(:,:,i)=kin.joint_T{i};
    end
    T=T*TT(:,:,i);
end
TX(:,:,nj+1)=T;


p=T(1:3,4);
R=T(1:3,1:3);
Jp=jacobian(p,Q);
Jr=JX;


% Generate model file
fprintf('Generatig %s for %s from %s\n',robot.m_name,robot.description,robot.urdf);

fid = fopen([robot.m_name '.m'],'w');
fprintf(fid,'function [p,R,J]=%s(q,tcp)\r\n',robot.m_name);
fprintf(fid,'%% %s Kinematic model for %s (generated from %s)\r\n',upper(robot.m_name),robot.description,robot.urdf);
fprintf(fid,'%% \r\n');
fprintf(fid,'%% Usage: \r\n');
fprintf(fid,'%%           [p,R,J]=kinmodel_%s(q,tcp)\r\n',robot.name);
fprintf(fid,'%%           [p,R,J]=kinmodel_%s(q)\r\n',robot.name);
fprintf(fid,'%% \r\n');
fprintf(fid,'%% Input: \r\n');
fprintf(fid,'%%           q   joint position (nj x 1) \r\n');
fprintf(fid,'%%           tcp tool center point \r\n');
fprintf(fid,'%% \r\n');
fprintf(fid,'%% Output:    \r\n');
fprintf(fid,'%%           p   task position [x y z] (3 x 1) \r\n');
fprintf(fid,'%%           R   rotational matrix (3 x 3) \r\n');
fprintf(fid,'%%           J   Jacobian matrix (6 x nj) \r\n');
fprintf(fid,'%% \r\n');
fprintf(fid,'%% \r\n');
fprintf(fid,'\r\n');
fprintf(fid,'%% Copyright (c) 2022 by IJS Leon Zlajpah \r\n');
fprintf(fid,'%% \r\n');
fprintf(fid,'\r\n');

for i=1:nj
    ij=find(kin.joint_ind==i);
    if kin.joint_rev(ij)==1
        fprintf(fid,'c%d=cos(q(%d));\r\n',i,i);
        fprintf(fid,'s%d=sin(q(%d));\r\n',i,i);
    else
        fprintf(fid,'q%d=q(%d);\r\n',i,i);
    end
end
fprintf(fid,'\r\n');

% angles
na=length(a_sym);
for i=1:na
    a=a_sym(i)/pi*2;
    if abs(a-round(a))<0.0001
        fprintf(fid,'ca%d=cos(%d*pi/2);\r\n',i,round(a));
        fprintf(fid,'sa%d=sin(%d*pi/2);\r\n',i,round(a));
    else
        fprintf(fid,'ca%d=cos(%f);\r\n',i,a_sym(i));
        fprintf(fid,'sa%d=sin(%f);\r\n',i,a_sym(i));
    end
end
fprintf(fid,'\r\n');

% distance
np=length(p_sym);
for i=1:np
    fprintf(fid,'p%d=%f;\r\n',i,p_sym(i));
end
fprintf(fid,'\r\n');

% p
fprintf(fid,'p=[...\r\n');
for i=1:3
    fprintf(fid,'    %s;...\r\n',subsincos(sprintf('%s',p(i)),nj,na));
end
fprintf(fid,'  ];\r\n');
fprintf(fid,'\r\n');

% R
fprintf(fid,'R=zeros(3,3);\r\n');
for i=1:3
    for j=1:3
        fprintf(fid,'R(%d,%d)=%s;\r\n',i,j,subsincos(sprintf('%s',R(i,j)),nj,na));
    end
end
fprintf(fid,'\r\n');

fprintf(fid,'Jp=zeros(3,%d);\r\n',nj);
for i=1:3
    for j=1:nj
        fprintf(fid,'Jp(%d,%d)=%s;\r\n',i,j,subsincos(sprintf('%s',Jp(i,j)),nj,na));
    end
end
fprintf(fid,'\r\n');

fprintf(fid,'Jr=zeros(3,%d);\r\n',nj);
for i=1:3
    for j=1:nj
        fprintf(fid,'Jr(%d,%d)=%s;\r\n',i,j,subsincos(sprintf('%s',Jr(i,j)),nj,na));
    end
end
fprintf(fid,'\r\n');

% TCP
fprintf(fid,'if nargin==2\r\n');
fprintf(fid,'    if isequal(size(tcp),[4 4])\r\n');
fprintf(fid,'        p_tcp=tcp(1:3,4);\r\n');
fprintf(fid,'        R_tcp=tcp(1:3,1:3);\r\n');
fprintf(fid,'    elseif isequal(size(tcp),[1 3])\r\n');
fprintf(fid,'        p_tcp=tcp(1:3)'';\r\n');
fprintf(fid,'        R_tcp=eye(3);\r\n');
fprintf(fid,'    elseif isequal(size(tcp),[3 1])\r\n');
fprintf(fid,'        p_tcp=tcp(1:3);\r\n');
fprintf(fid,'        R_tcp=eye(3);\r\n');
fprintf(fid,'    elseif isequal(size(tcp),[1 7])\r\n');
fprintf(fid,'        p_tcp=tcp(1:3)'';\r\n');
fprintf(fid,'        R_tcp=q2r(tcp(4:7));\r\n');
fprintf(fid,'    elseif isequal(size(tcp),[7 1])\r\n');
fprintf(fid,'        p_tcp=tcp(1:3);\r\n');
fprintf(fid,'        R_tcp=q2r(tcp(4:7));\r\n');
fprintf(fid,'    elseif isequal(size(tcp),[1 6])\r\n');
fprintf(fid,'        p_tcp=tcp(1:3)'';\r\n');
fprintf(fid,'        R_tcp=rpy2r(tcp(4:6));\r\n');
fprintf(fid,'    elseif isequal(size(tcp),[6 1])\r\n');
fprintf(fid,'        p_tcp=tcp(1:3);\r\n');
fprintf(fid,'        R_tcp=rpy2r(tcp(4:6));\r\n');
fprintf(fid,'    else\r\n');
fprintf(fid,'        error(''kinmodel: wrong tcp form'')\r\n');
fprintf(fid,'    end\r\n');
fprintf(fid,'\r\n');
fprintf(fid,'    p=p+R*p_tcp;\r\n');
fprintf(fid,'    Jp=Jp+v2s(R*p_tcp)''*Jr;\r\n');
fprintf(fid,'    R=R*R_tcp;\r\n');
fprintf(fid,'end\r\n');
fprintf(fid,'\r\n');

fprintf(fid,'J=[Jp;Jr];\r\n');
fprintf(fid,'\r\n');

fclose(fid);



end

function  [T]=joint_transform(joint_typ,q)
switch joint_typ
    case 'Rx'             % revolute X axis
        T=rp2t(rot_x(q));
    case 'Ry'             % revolute Y axis
        T=rp2t(rot_y(q));
    case {'R','Rz'}       % revolute Z axis
        T=rp2t(rot_z(q));
    case 'Px'             % prismatic X axis
        T=rp2t([q 0 0]);
    case 'Py'             % prismatic Y axis
        T=rp2t([0 q 0]);
    case {'P','Pz'}       % prismatic Z axis
        T=rp2t([0 0 q]);
    case '-Rx'             % revolute X axis
        T=rp2t(rot_x(-q));
    case '-Ry'             % revolute Y axis
        T=rp2t(rot_y(-q));
    case {'-R','-Rz'}       % revolute Z axis
        T=rp2t(rot_z(-q));
    case '-Px'             % prismatic X axis
        T=rp2t([-q 0 0]);
    case '-Py'             % prismatic Y axis
        T=rp2t([0 -q 0]);
    case {'-P','-Pz'}       % prismatic Z axis
        T=rp2t([0 0 -q]);
    case 'F'              % fixed
        T=eye(4);
    otherwise
        error( 'Unsuppoerted joint type ''%s''',joint_typ);
end
end

function  [a]=joint_axis(joint_typ)
switch joint_typ
    case 'Rx'             % revolute X axis
        a=[1 0 0]';
    case 'Ry'             % revolute Y axis
        a=[0 1 0]';
    case {'R','Rz'}       % revolute Z axis
        a=[0 0 1]';
    case '-Rx'             % revolute X axis
        a=[-1 0 0]';
    case '-Ry'             % revolute Y axis
        a=[0 -1 0]';
    case {'-R','-Rz'}       % revolute Z axis
        a=[0 0 -1]';
    case {'Px','Py','P','Pz','-Px','-Py','-P','-Pz'}       % prismatic
        a=[0 0 0]';
    case 'F'              % fixed
        a=[0 0 0]';
    otherwise
        error( 'Unsuppoerted joint type ''%s''',joint_typ);
end
end

function s=subsincos(s,n,na)

for i=1:n
    s=strrep(s,sprintf('cos(q%d)',i),sprintf('c%d',i));
    s=strrep(s,sprintf('sin(q%d)',i),sprintf('s%d',i));
end   

for i=1:na
    s=strrep(s,sprintf('cos(a%d)',i),sprintf('ca%d',i));
    s=strrep(s,sprintf('sin(a%d)',i),sprintf('sa%d',i));
end 
end

