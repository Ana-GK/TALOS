
%% This program saves position (joints and cartesic coordinates) of the robot to the Excel file

clear all;

file_name = 'default_pose.xlsx';
file_path = '/home/cobotat4/Documents/MATLAB/Talos_keyboard/';

file = strcat(file_path, file_name);

% Initialize ROS

rosshutdown;
pause(1);

rosinit;
pause(1);

% Initialize global variables

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

disp("Global variables initialized")

pause(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Read initial joint states

global sub_states;
global msg_states;

sub_states = rossubscriber('/joint_states');
msg_states = receive(sub_states, 1); % 1 is timeout in seconds

global initial_joint_positions;
global initial_joint_names;

initial_joint_positions = msg_states.Position; % Save initial joint positions
initial_joint_names = msg_states.Name; % Save joint names

disp(initial_joint_positions)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Error codes - CREATE DICTIONARIES TO LOOKUP FK ERRORS

global d_fk;

mes = rosmessage('moveit_msgs/GetPositionFKResponse');
val = mes.ErrorCode;

d_fk = containers.Map;

props = properties(val);

for prop = 4:length(props)-1
    thisprop = props{prop};
    % d(key) = value
    value = thisprop;
%     disp(value);
    key = string(val.(thisprop));
%     disp(key);
    d_fk(key) = value;
end


%%%%%%%%%%%%%%%%%%
% Forward kinematics
%%%%%%%%%%%%%%%%%%

% variables for FK

left_arm_joint_positions = initial_joint_positions(1:7);
left_arm_joint_names = initial_joint_names(1:7);

right_arm_joint_positions = initial_joint_positions(8:14);
right_arm_joint_names = initial_joint_names(8:14);


% Create client for rosservice /compute_fk
fk_client = rossvcclient("/compute_fk","DataFormat","struct"); % create a client


% Fill the message with necessary data

%%% LEFT ARM

% Create message of the correct type for this service
fk_req_LArm = rosmessage(fk_client); % create a message

fk_req_LArm.Header.FrameId = 'base_link'; % using coordinate system of the base
fk_req_LArm.FkLinkNames = {'arm_left_1_link', 'arm_left_2_link', 'arm_left_3_link', 'arm_left_4_link', 'arm_left_5_link', 'arm_left_6_link', 'arm_left_7_link'};

fk_req_LArm.RobotState.JointState.Name = left_arm_joint_names;
fk_req_LArm.RobotState.JointState.Position = left_arm_joint_positions;

%%% RIGHT ARM

% Create message of the correct type for this service
fk_req_RArm = rosmessage(fk_client);

fk_req_RArm.Header.FrameId = 'base_link';
fk_req_RArm.FkLinkNames = {'arm_right_1_link', 'arm_right_2_link', 'arm_right_3_link', 'arm_right_4_link', 'arm_right_5_link', 'arm_right_6_link', 'arm_right_7_link'};

fk_req_RArm.RobotState.JointState.Name = right_arm_joint_names;
fk_req_RArm.RobotState.JointState.Position = right_arm_joint_positions;

%%% Send the message and wait for response

if isServerAvailable(fk_client)
    fk_resp_LArm = call(fk_client,fk_req_LArm,"Timeout",10);
    fk_resp_RArm = call(fk_client,fk_req_RArm,"Timeout",10);
else
    error("Service server not available on network")
end

% Check if error has occured

if (fk_resp_LArm.ErrorCode.Val ~= 1) | (fk_resp_RArm.ErrorCode.Val ~= 1)
    error_name = d_fk(string(fk_resp_RArm.ErrorCode.Val));
    fprintf("FK error, value: %d, name: %s\n",fk_resp_RArm.ErrorCode.Val,error_name)
end

% Sort the data from fk response

link_names = [fk_resp_LArm.FkLinkNames fk_resp_RArm.FkLinkNames];
coordinates = [fk_resp_LArm.PoseStamped fk_resp_RArm.PoseStamped]; % link states relative to base_link!!!

global num_limbs
num_limbs = 2; % Left and right arm

initial_cart_position = [];

 % Cartesic positions for both arms (left arm has first coordinate 1, right arm has first coordinate 2)

for i=1:num_limbs
    initial_cart_position(i,1) = coordinates(7,i).Pose.Position.X;
    initial_cart_position(i,2) = coordinates(7,i).Pose.Position.Y;
    initial_cart_position(i,3) = coordinates(7,i).Pose.Position.Z;
    initial_cart_position(i,4) = coordinates(7,i).Pose.Orientation.X;
    initial_cart_position(i,5) = coordinates(7,i).Pose.Orientation.Y;
    initial_cart_position(i,6) = coordinates(7,i).Pose.Orientation.Z;
    initial_cart_position(i,7) = coordinates(7,i).Pose.Orientation.W;
end


% Create Table

joint_names = initial_joint_names;
joint_values = initial_joint_positions;
L_arm_EE_coordinates = [{'x'}; {'y'}; {'z'}; {'q_x'}; {'q_y'}; {'q_z'}; {'q_w'}];
L_arm_EE_values = initial_cart_position(1,:)';
R_arm_EE_coordinates = [{'x'}; {'y'}; {'z'}; {'q_x'}; {'q_y'}; {'q_z'}; {'q_w'}];
R_arm_EE_values = initial_cart_position(2,:)';

default_joint_position = table(joint_names, joint_values);
default_cart_position = table(L_arm_EE_coordinates, L_arm_EE_values, R_arm_EE_coordinates, R_arm_EE_values);

disp(joint_names)
disp(joint_values)
disp(L_arm_EE_coordinates)
disp(L_arm_EE_values)
disp(R_arm_EE_coordinates)
disp(R_arm_EE_values)

disp(default_joint_position)

disp(default_cart_position)

% Write to Excel table

writetable(default_joint_position,file_name,'Sheet',1,'Range','A1')

writetable(default_cart_position,file,'Sheet',1,'Range','C1')













