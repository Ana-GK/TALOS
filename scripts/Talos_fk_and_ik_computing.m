
clear all; % IMPORTANT!!! (because it sometimes does't work without clear all)

% Initialize ROS
rosinit;

%%%%%%%%%%%%%%%%%%

global sub_states;
global msg_states;

sub_states = rossubscriber('/joint_states');
msg_states = receive(sub_states, 1); % 1 is timeout in seconds

% Read initial joint positions

all_joint_positions = msg_states.Position;

% LArm = all_joint_positions(1:7);
% RArm = all_joint_positions(8:14);
% 
% LLeg = all_joint_positions(19:24);
% RLeg = all_joint_positions(25:30);
% 
% Head = all_joint_positions(17:18);
% Torso = all_joint_positions(31:32);

% Read joint positions only for left arm

left_arm_joint_positions = msg_states.Position(1:7);
left_arm_joint_names = msg_states.Name(1:7);

%%%%%%%%%%%%%%%%%%
% Forward kinematics
%%%%%%%%%%%%%%%%%%

fk_client = rossvcclient("/compute_fk","DataFormat","struct");

fk_req = rosmessage(fk_client);

fk_req.Header.FrameId = 'base_link';
fk_req.FkLinkNames = {'arm_left_1_link', 'arm_left_2_link', 'arm_left_3_link', 'arm_left_4_link', 'arm_left_5_link', 'arm_left_6_link', 'arm_left_7_link'};

fk_req.RobotState.JointState.Name = left_arm_joint_names;
fk_req.RobotState.JointState.Position = left_arm_joint_positions;

if isServerAvailable(fk_client)
    fk_resp = call(fk_client,fk_req,"Timeout",10);
else
    error("Service server not available on network")
end


if fk_resp.ErrorCode.Val ~= 1
    fprintf("FK error, code is %d", ik_resp.ErrorCode.Val)
end

link_names = fk_resp.FkLinkNames;
coordinates = fk_resp.PoseStamped; % link states relative to base_link!!!

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

cart_position = [];
cart_position = [cart_position coordinates(7).Pose.Position.X];
cart_position = [cart_position coordinates(7).Pose.Position.Y];
cart_position = [cart_position coordinates(7).Pose.Position.Z];
cart_position = [cart_position coordinates(7).Pose.Orientation.X];
cart_position = [cart_position coordinates(7).Pose.Orientation.Y];
cart_position = [cart_position coordinates(7).Pose.Orientation.Z];
cart_position = [cart_position coordinates(7).Pose.Orientation.W];


old_cart_position = cart_position;

% change cartesian positions (x-1, y-2, z-3)
cart_position(2) = cart_position(2)-0.05;


%%%%%%%%%%%%%%%%%%
% Inverse kinematics
%%%%%%%%%%%%%%%%%%

ik_client = rossvcclient("/compute_ik","DataFormat","struct");

ik_req = rosmessage(ik_client);

% ik_req.IkRequest.Timeout.Sec = int32(5);

ik_req.IkRequest.RobotState.JointState.Position = left_arm_joint_positions;
ik_req.IkRequest.RobotState.JointState.Name = left_arm_joint_names;

ik_req.IkRequest.GroupName = 'left_arm';
ik_req.IkRequest.IkLinkName = 'arm_left_7_link';

ik_req.IkRequest.PoseStamped.Header.FrameId = 'base_link';


ik_req.IkRequest.PoseStamped.Pose.Position.X = cart_position(1);
ik_req.IkRequest.PoseStamped.Pose.Position.Y = cart_position(2);
ik_req.IkRequest.PoseStamped.Pose.Position.Z = cart_position(3);

ik_req.IkRequest.PoseStamped.Pose.Orientation.X = cart_position(4);
ik_req.IkRequest.PoseStamped.Pose.Orientation.Y = cart_position(5);
ik_req.IkRequest.PoseStamped.Pose.Orientation.Z = cart_position(6);
ik_req.IkRequest.PoseStamped.Pose.Orientation.W = cart_position(7);



if isServerAvailable(ik_client)
    ik_resp = call(ik_client,ik_req,"Timeout",10);
else
    error("Service server not available on network")
end


if ik_resp.ErrorCode.Val ~= 1
    fprintf("IK error, code is %d", ik_resp.ErrorCode.Val)
end

joints = ik_resp.Solution.JointState.Position;


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

LArm = joints(15:21);

%%%%%%
LArm = joints(15:21);
RArm = joints(29:35);

LLeg = joints(1:6);
RLeg = joints(7:12);

Head = joints(43:44);
Torso = joints(13:14);
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

disp("LArm Joint positions (before FK):")
disp(left_arm_joint_positions)
disp("LArm Cartesian positions (from FK):")
disp(old_cart_position)
disp("LArm Cartesian positions (changed):")
disp(cart_position)
disp("LArm Joint positions (from IK):")
disp(LArm)

