clear all; % IMPORTANT!!! (because it sometimes does't work without clear all)

rosshutdown
pause(5);
% Initialize ROS
rosinit

%%%%%%%%%%%%%%%%%%

global sub_states;
global msg_states;

sub_states = rossubscriber('/joint_states');
msg_states = receive(sub_states, 1); % 1 is timeout in seconds

% Read initial joint positions

all_joint_positions = msg_states.Position;

LArm = all_joint_positions(1:7);
RArm = all_joint_positions(8:14);

LLeg = all_joint_positions(19:24);
RLeg = all_joint_positions(25:30);

Head = all_joint_positions(17:18);
Torso = all_joint_positions(31:32);

% Read joint positions only for left arm

left_arm_joint_positions = msg_states.Position(1:7);
left_arm_joint_names = msg_states.Name(1:7);

right_arm_joint_positions = msg_states.Position(8:14);
right_arm_joint_names = msg_states.Name(8:14);

%%%%%%%%%%%%%%%%%%
% Forward kinematics
%%%%%%%%%%%%%%%%%%

fk_client = rossvcclient("/compute_fk","DataFormat","struct");

fk_req_LArm = rosmessage(fk_client);

fk_req_LArm.Header.FrameId = 'base_link';
fk_req_LArm.FkLinkNames = {'arm_left_1_link', 'arm_left_2_link', 'arm_left_3_link', 'arm_left_4_link', 'arm_left_5_link', 'arm_left_6_link', 'arm_left_7_link'};

fk_req_LArm.RobotState.JointState.Name = left_arm_joint_names;
fk_req_LArm.RobotState.JointState.Position = left_arm_joint_positions;

%%%

fk_req_RArm = rosmessage(fk_client);

fk_req_RArm.Header.FrameId = 'base_link';
fk_req_RArm.FkLinkNames = {'arm_right_1_link', 'arm_right_2_link', 'arm_right_3_link', 'arm_right_4_link', 'arm_right_5_link', 'arm_right_6_link', 'arm_right_7_link'};

fk_req_RArm.RobotState.JointState.Name = right_arm_joint_names;
fk_req_RArm.RobotState.JointState.Position = right_arm_joint_positions;

%%%

if isServerAvailable(fk_client)
    fk_resp_LArm = call(fk_client,fk_req_LArm,"Timeout",10);
    fk_resp_RArm = call(fk_client,fk_req_RArm,"Timeout",10);
else
    error("Service server not available on network")
end


if (fk_resp_LArm.ErrorCode.Val ~= 1) | (fk_resp_RArm.ErrorCode.Val ~= 1)
    fprintf("FK error \n")
end

% link_names = fk_resp.FkLinkNames;
% coordinates = fk_resp.PoseStamped; % link states relative to base_link!!!

link_names = [fk_resp_LArm.FkLinkNames fk_resp_RArm.FkLinkNames];
coordinates = [fk_resp_LArm.PoseStamped fk_resp_RArm.PoseStamped]; % link states relative to base_link!!!

x_coordinates = [];
y_coordinates = [];
z_coordinates = [];

disp("x")
for i=1:length(coordinates)
    x_coordinates = [x_coordinates coordinates(i).Pose.Position.X];
end
% disp(x_coordinates)

disp("y")
for i=1:length(coordinates)
    y_coordinates = [y_coordinates coordinates(i).Pose.Position.Y];
end
% disp(y_coordinates)

disp("z")
for i=1:length(coordinates)
    z_coordinates = [z_coordinates coordinates(i).Pose.Position.Z];
end
% disp(z_coordinates)

num_limbs = 2;

cart_position = [];

for i=1:num_limbs
    cart_position(i,1) = coordinates(7,i).Pose.Position.X;
    cart_position(i,2) = coordinates(7,i).Pose.Position.Y;
    cart_position(i,3) = coordinates(7,i).Pose.Position.Z;
    cart_position(i,4) = coordinates(7,i).Pose.Orientation.X;
    cart_position(i,5) = coordinates(7,i).Pose.Orientation.Y;
    cart_position(i,6) = coordinates(7,i).Pose.Orientation.Z;
    cart_position(i,7) = coordinates(7,i).Pose.Orientation.W;
end

old_cart_position = cart_position;

% change cartesian positions (first coord. 1 - left, 2 - right) (sec. coord. x-1, y-2, z-3)

% % hands back
% cart_position(1,1) = cart_position(1,1)-0.05;
% cart_position(2,1) = cart_position(2,1)-0.05;

% % hands forward
% cart_position(1,1) = cart_position(1,1)+0.05;
% cart_position(2,1) = cart_position(2,1)+0.05;


% % hands towards the body
% cart_position(1,2) = cart_position(1,2)-0.05;
% cart_position(2,2) = cart_position(2,2)+0.05;

% hands away from the body
% cart_position(1,2) = cart_position(1,2)+0.05;
% cart_position(2,2) = cart_position(2,2)-0.05;


% % hands up
% cart_position(1,3) = cart_position(1,3)+0.05;
% cart_position(2,3) = cart_position(2,3)+0.05;

% % hands down
cart_position(1,3) = cart_position(1,3)-0.05;
cart_position(2,3) = cart_position(2,3)-0.05;




%%%%%%%%%%%%%%%%%%
% Inverse kinematics
%%%%%%%%%%%%%%%%%%

ik_client = rossvcclient("/compute_ik","DataFormat","struct");

ik_req_LArm = rosmessage(ik_client);

% ik_req.IkRequest.Timeout.Sec = int32(5);

ik_req_LArm.IkRequest.RobotState.JointState.Position = left_arm_joint_positions;
ik_req_LArm.IkRequest.RobotState.JointState.Name = left_arm_joint_names;

ik_req_LArm.IkRequest.GroupName = 'left_arm';
ik_req_LArm.IkRequest.IkLinkName = 'arm_left_7_link';

ik_req_LArm.IkRequest.PoseStamped.Header.FrameId = 'base_link';


ik_req_LArm.IkRequest.PoseStamped.Pose.Position.X = cart_position(1,1);
ik_req_LArm.IkRequest.PoseStamped.Pose.Position.Y = cart_position(1,2);
ik_req_LArm.IkRequest.PoseStamped.Pose.Position.Z = cart_position(1,3);

ik_req_LArm.IkRequest.PoseStamped.Pose.Orientation.X = cart_position(1,4);
ik_req_LArm.IkRequest.PoseStamped.Pose.Orientation.Y = cart_position(1,5);
ik_req_LArm.IkRequest.PoseStamped.Pose.Orientation.Z = cart_position(1,6);
ik_req_LArm.IkRequest.PoseStamped.Pose.Orientation.W = cart_position(1,7);

%%%

ik_req_RArm = rosmessage(ik_client);

% ik_req.IkRequest.Timeout.Sec = int32(5);

ik_req_RArm.IkRequest.RobotState.JointState.Position = right_arm_joint_positions;
ik_req_RArm.IkRequest.RobotState.JointState.Name = right_arm_joint_names;

ik_req_RArm.IkRequest.GroupName = 'right_arm';
ik_req_RArm.IkRequest.IkLinkName = 'arm_right_7_link';

ik_req_RArm.IkRequest.PoseStamped.Header.FrameId = 'base_link';


ik_req_RArm.IkRequest.PoseStamped.Pose.Position.X = cart_position(2,1);
ik_req_RArm.IkRequest.PoseStamped.Pose.Position.Y = cart_position(2,2);
ik_req_RArm.IkRequest.PoseStamped.Pose.Position.Z = cart_position(2,3);

ik_req_RArm.IkRequest.PoseStamped.Pose.Orientation.X = cart_position(2,4);
ik_req_RArm.IkRequest.PoseStamped.Pose.Orientation.Y = cart_position(2,5);
ik_req_RArm.IkRequest.PoseStamped.Pose.Orientation.Z = cart_position(2,6);
ik_req_RArm.IkRequest.PoseStamped.Pose.Orientation.W = cart_position(2,7);


if isServerAvailable(ik_client)
    ik_resp_LArm = call(ik_client,ik_req_LArm,"Timeout",10);
    ik_resp_RArm = call(ik_client,ik_req_RArm,"Timeout",10);
else
    error("Service server not available on network")
end


if (ik_resp_LArm.ErrorCode.Val) ~= 1 | (ik_resp_RArm.ErrorCode.Val ~= 1)
    fprintf("IK error \n")
end

LArm = ik_resp_LArm.Solution.JointState.Position(15:21);
RArm = ik_resp_RArm.Solution.JointState.Position(29:35);


%%%%%%%%%%%%%%%%%%
% Publish new joint positions
%%%%%%%%%%%%%%%%%%

global pub_Lleg;
global msg_Lleg;
global pub_Rleg;
global msg_Rleg;
global pub_Larm;
global msg_Larm;
global pub_Rarm;
global msg_Rarm;
global pub_torso;
global msg_torso;
global pub_head;
global msg_head;

global msgP_Lleg;
global msgP_Rleg;
global msgP_Larm;
global msgP_Rarm;
global msgP_torso;
global msgP_head;

[pub_Lleg,msg_Lleg] = rospublisher('/left_leg_controller/command','trajectory_msgs/JointTrajectory');
[pub_Rleg,msg_Rleg] = rospublisher('/right_leg_controller/command','trajectory_msgs/JointTrajectory');
[pub_Larm,msg_Larm] = rospublisher('/left_arm_controller/command','trajectory_msgs/JointTrajectory');
[pub_Rarm,msg_Rarm] = rospublisher('/right_arm_controller/command','trajectory_msgs/JointTrajectory');
[pub_torso,msg_torso] = rospublisher('/torso_controller/command','trajectory_msgs/JointTrajectory');
[pub_head,msg_head] = rospublisher('/head_controller/command','trajectory_msgs/JointTrajectory');

msgP_Lleg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_Rleg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_Larm = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_Rarm = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_torso = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_head = rosmessage('trajectory_msgs/JointTrajectoryPoint');

msg_Lleg.JointNames = {'leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint'};
msg_Rleg.JointNames = {'leg_right_1_joint', 'leg_right_2_joint', 'leg_right_3_joint', 'leg_right_4_joint', 'leg_right_5_joint', 'leg_right_6_joint'};
msg_Larm.JointNames = {'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'};
msg_Rarm.JointNames = {'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'};
msg_torso.JointNames = {'torso_1_joint', 'torso_2_joint'};
msg_head.JointNames = {'head_1_joint', 'head_2_joint'};

% Divide joint positions in groups by limbs

%%%%%%
% LArm = joints(15:21);
% RArm = joints(29:35);
% 
% LLeg = joints(1:6);
% RLeg = joints(7:12);
% 
% Head = joints(43:44);
% Torso = joints(13:14);
%%%%%%

N = 1;

for i=1:N

    dT1 = 0.5;
    
    msgP_Lleg.Positions = [LLeg]';
    msgP_Lleg.TimeFromStart = ros.msg.Duration(dT1);   
    msg_Lleg.Points(1) = msgP_Lleg;
    msgP_Rleg.Positions = [RLeg]';
    msgP_Rleg.TimeFromStart = ros.msg.Duration(dT1);   
    msg_Rleg.Points(1) = msgP_Rleg;
    msgP_Larm.Positions = [LArm]';
    msgP_Larm.TimeFromStart = ros.msg.Duration(dT1);   
    msg_Larm.Points(1) = msgP_Larm;
    msgP_Rarm.Positions = [RArm]';
    msgP_Rarm.TimeFromStart = ros.msg.Duration(dT1);   
    msg_Rarm.Points(1) = msgP_Rarm;
    msgP_head.Positions = [Head]';
    msgP_head.TimeFromStart = ros.msg.Duration(dT1);   
    msg_head.Points(1) = msgP_head;
    msgP_torso.Positions = [Torso]';
    msgP_torso.TimeFromStart = ros.msg.Duration(dT1);   
    msg_torso.Points(1) = msgP_torso;
    
    send(pub_Lleg,msg_Lleg);
    send(pub_Rleg,msg_Rleg);
    send(pub_Larm,msg_Larm);
    send(pub_Rarm,msg_Rarm);
    send(pub_head,msg_head);
    send(pub_torso,msg_torso);


end

rosshutdown

disp("LArm and RArm Joint positions (before FK):")
disp(left_arm_joint_positions)
disp(right_arm_joint_positions)
disp("LArm and RArm Cartesian positions (from FK):")
disp(old_cart_position)
disp("LArm and RArm Cartesian positions (changed):")
disp(cart_position)
disp("LArm and RArm Joint positions (from IK):")
disp(LArm)
disp(RArm)

