
%% Program for force dependent movement of Talos (INTEGRATES FORCES)

% Speed of the movement depends on the size of the force applied.

% New joint positions are calculated in every iteration of the while loop from
% current joint positions - which are actually previously calculated joint
% positions, so that it doesn't read joint states in every loop

% Current cartesic position is calculated and saved in every iteration (from
% the "integral")
% (forwards kinematics - {joints to x,y,z} - is calculated only once)

% If 90% of the joint limit is reached while Talos is moving, the movement
% is stopped
% Talos also has defined work space in cartesic coordinates in which it can
% move.


% DIRECTION OF FORCES
% x direction: Talos moves forwards(+) and backwards(-)
% y direction: Talos moves right(+) and left(-)
% z direction: Talos moves up(+) and down(-)

%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIRED MOVEMENT; direction of aplied force to: L_GRIPPER and R_GRIPPER:

% FORWARD L:+x, R:+x
% BACK L:-x, R-x:
% RIGHT L:+y, R:+y
% LEFT L:-y, R:-y
% UP L:+z, R:+z
% DOWN L:-z, R:-z
%%%%%%%%%%%%%%%%%%%%%%%%%


% initialize ROS

clear all;

rosshutdown;
pause(1);

rosinit;
pause(1);

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

global sub_l;
global sub_r;
global msg_l;
global msg_r;

disp("Global variables initialized")

pause(1);

sub_r = rossubscriber('/right_wrist_ft');
pause(2);
msg_r = receive(sub_r, 5);

disp("Received data for right wrist")

sub_l = rossubscriber('/left_wrist_ft');
pause(2);
msg_l = receive(sub_l, 5);

disp("Received data for left wrist")

% force offsets in the wrists
global offset_LX;
global offset_RX;
global offset_LY;
global offset_RY;
global offset_LZ;
global offset_RZ;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the force offests
fprintf('Get the force offsets\n')

N_offset = 50; % number of measurements to average
tab_offset_LX = nan(1,N_offset);
tab_offset_RX = nan(1,N_offset);
tab_offset_LY = nan(1,N_offset);
tab_offset_RY = nan(1,N_offset);
tab_offset_RZ = nan(1,N_offset);
tab_offset_LZ = nan(1,N_offset);

for i = 1:N_offset
    msg_r = receive(sub_r, 5);
    msg_l = receive(sub_l, 5);
    tab_offset_LX(1,i) = msg_l.Wrench.Force.X;
    tab_offset_RX(1,i) = msg_r.Wrench.Force.X;
    tab_offset_LY(1,i) = msg_l.Wrench.Force.Y;
    tab_offset_RY(1,i) = msg_r.Wrench.Force.Y;
    tab_offset_LZ(1,i) = msg_l.Wrench.Force.Z;
    tab_offset_RZ(1,i) = msg_r.Wrench.Force.Z;
    pause(0.05);
end

offset_LX = sum(tab_offset_LX) / N_offset;
offset_RX = sum(tab_offset_RX) / N_offset;
offset_LY = sum(tab_offset_LY) / N_offset;
offset_RY = sum(tab_offset_RY) / N_offset;
offset_LZ = sum(tab_offset_LZ) / N_offset;
offset_RZ = sum(tab_offset_RZ) / N_offset;

disp(offset_LX)
disp(offset_RX)
disp(offset_LY)
disp(offset_RY)
disp(offset_LZ)
disp(offset_RZ)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The controll window

global swi1;

fig = uifigure;
swi1 = uiswitch(fig);
swi1.Position = [80 350 45 20];
swi1.Value = 'On';
uilabel(fig,'Position',[180 350 200 20],'Text','Stop the simulation');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Error codes - CREATE DICTIONARIES TO LOOKUP FK AND IK ERRORS

global d_fk;
global d_ik;

mes = rosmessage('moveit_msgs/GetPositionFKResponse');
val = mes.ErrorCode;

d_fk = containers.Map;

props = properties(val);

for prop = 4:length(props)-1
    thisprop = props{prop};
    % d(key) = value
    value = thisprop;
    disp(value)
    key = string(val.(thisprop));
    disp(key)
    d_fk(key) = value;
end

mes = rosmessage('moveit_msgs/GetPositionIKResponse');
val = mes.ErrorCode;

d_ik = containers.Map;

props = properties(val);

for prop = 4:length(props)-1
    thisprop = props{prop};
    % d(key) = value
    value = thisprop;
    disp(value)
    key = string(val.(thisprop));
    disp(key)
    d_ik(key) = value;
end

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

% Divide joint positions in groups by limbs

LArm = initial_joint_positions(1:7);
RArm = initial_joint_positions(8:14);

LLeg = initial_joint_positions(19:24);
RLeg = initial_joint_positions(25:30);

Head = initial_joint_positions(17:18);
Torso = initial_joint_positions(31:32);

% Save the 7th joint, the value is used after the inverse kinematics, to
% correct the angle of the 7th joint (it stays locked on ther same value)

initial_joint_position_arm_left_7_joint = LArm(7);
initial_joint_position_arm_right_7_joint = RArm(7);


% Lowest value of applied force for Talos to move
lowest_F_value = 20;

previous_joints = initial_joint_positions; % array for saving previous joint positions

% Joint limits
global lower_limit;
global upper_limit;
global new_lower_limit;
global new_upper_limit;

lower_limit = [-1.57,0.01,-2.43,-2.23,-2.51,-1.37,-0.68,-0.79,-2.87,-2.43,-2.23,-2.51,-1.37,-0.68,-0.96,-0.96,-0.21,-1.31,-0.35,-0.52,-2.10,-0.00,-1.27,-0.52,-1.57,-0.52,-2.10,0.00,-1.27,-0.52,-1.26,-0.23];
upper_limit = [0.79,2.87,2.43,0.00,2.51,1.37,0.68,1.57,-0.01,2.43,0.00,2.51,1.37,0.68,0.00,0.00,0.79,1.31,1.57,0.52,0.70,2.62,0.68,0.52,0.35,0.52,0.70,2.62,0.68,0.52,1.26,0.73];

% new limit is 90% of actual limit
new_upper_limit = upper_limit - upper_limit*0.1;
new_lower_limit = lower_limit - lower_limit*0.1;

new_upper_limit = new_upper_limit';
new_lower_limit = new_lower_limit';

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


% Cart limits
% upper limit is limit in the positive direction of the axis
% lower limit is the limit in the negative direction of the axis
global x_up_limit
global x_low_limit
global y_up_limit
global y_low_limit
global z_up_limit
global z_low_limit

x_up_limit = initial_cart_position(1,1) + 0.10;
x_low_limit = initial_cart_position(1,1) - 0.07;

y_up_limit(1) = initial_cart_position(1,2) + 0.10;
y_up_limit(2) = initial_cart_position(2,2) + 0.10;
y_low_limit(1) = initial_cart_position(1,2) - 0.10;
y_low_limit(2) = initial_cart_position(2,2) - 0.10;

z_up_limit = initial_cart_position(1,3) + 0.15;
z_low_limit = initial_cart_position(1,3) - 0.10;



i = 0; % for counting iterations
quasi_integral = [0 0 0]; % "integral" made with addition
all_q_integrals = [0 0 0]; % saves all previous "integrals"
current_cart_position = initial_cart_position; % for the first iteration of the while loop

delta_x = 0; % initialize variables delta
delta_y = 0;
delta_z = 0;

all_cart_positions_L = [];
all_cart_positions_R = [];
F_x_all = [];
all_joints = [];
all_read_joints = [];
initial_joints = [LArm' RArm'];
all_F_values = [];
F_values_for_plot = zeros(3000,3);

previous_joints = msg_states;

r = rosrate(10);

K = 0.00015; % constant for integral

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while strcmp(swi1.Value,'On') % While the button is on

    i = i+1;

    msg_r = receive(sub_r, 1); % 1 is timeout in seconds
    msg_l = receive(sub_l, 1); % 1 is timeout in seconds

    F_x_r = msg_r.Wrench.Force.X - offset_RX;
    F_y_r = msg_r.Wrench.Force.Y - offset_RY;
    F_z_r = msg_r.Wrench.Force.Z - offset_RZ;

    F_x_l = msg_l.Wrench.Force.X - offset_LX;
    F_y_l = msg_l.Wrench.Force.Y - offset_LY;
    F_z_l = msg_l.Wrench.Force.Z - offset_LZ;

    F_x = -F_z_l - F_z_r; % if F_x is + Talos moves forward, if F_x is - Talos moves back
    F_y = F_x_l - F_x_r; % if F_y is + Talos moves right, if F_y is - Talos moves left
    F_z = -F_y_l + F_y_r; % if F_z is + Talos moves up, if F_z is - Talos moves down

    F_values_for_plot(i,:) = [F_x F_y F_z];

    % Find direction of the largest force

    [F_value,F_index] = max([abs(F_x),abs(F_y),abs(F_z)]); % F_value is absolute value of the largest force
    % F_index is the number (1-3) that tells which force is the largest

    % All other forces except the largest are set to zero (to prevent
    % interferance)    
    if F_index == 1
        F_y = 0;
        F_z = 0;
    elseif F_index == 2
        F_x = 0;
        F_z = 0;
    else
        F_x = 0;
        F_y = 0;
    end


    all_F_values = [all_F_values F_value];

    % Save the forces in a 2d array all_forces:
    forces_Talos = [F_x F_y F_z];

%     msg_states = receive(sub_states, 1); % 1 is timeout in seconds
%     joint_positions = msg_states; % Save joint positions

    % Positions for when force is lower than lowest_F_value
    LArm = previous_joints.Position(1:7);
    RArm = previous_joints.Position(8:14);

    read_joints = [LArm' RArm'];
    all_read_joints = [all_read_joints; read_joints];

    new_cart_position = current_cart_position;

    % Pogledam če je pozicija Talosa v kartezičnih koordinatah izven
    % delovnega prostora
    % V primeru gre čez mejo, new_cart_position spet postavim na
    % current_cart_position

    if F_value > lowest_F_value

%         quasi_integral = quasi_integral + forces_Talos*K;
% 
%         all_q_integrals = [all_q_integrals; quasi_integral];
%     
%         delta_x = quasi_integral(1); % size of the move in x direction
%         delta_y = quasi_integral(2);
%         delta_z = quasi_integral(3);
        
%         new_cart_position = initial_cart_position;

%         for i=1:num_limbs
%             new_cart_position(i,1) = new_cart_position(i,1) + delta_x;
%             new_cart_position(i,2) = new_cart_position(i,2) + delta_y;
%             new_cart_position(i,3) = new_cart_position(i,3) + delta_z;
%         end

        for i=1:num_limbs
            new_cart_position(i,1) = new_cart_position(i,1) + forces_Talos(1)*K;
            new_cart_position(i,2) = new_cart_position(i,2) + forces_Talos(2)*K;
            new_cart_position(i,3) = new_cart_position(i,3) + forces_Talos(3)*K;
%             new_cart_position(i,4) = new_cart_position(i,4) + F_value*K;
%             new_cart_position(i,5) = new_cart_position(i,5) + F_value*K;
%             new_cart_position(i,6) = new_cart_position(i,6) + F_value*K;
%             new_cart_position(i,7) = new_cart_position(i,7) + F_value*K;
        end
    else
        new_joints = previous_joints;
    end

    stop = inside_work_zone(new_cart_position);
    
    if stop == 1
        new_cart_position = current_cart_position;
    end

    

%     F_x_all = [F_x_all F_x];

%     disp("Force x:")
%     disp(F_x)
%     disp("Force y:")
%     disp(F_y)
%     disp("Force z:")
%     disp(F_z)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % x direction (forward, back) - both arms together

    if abs(F_x) > lowest_F_value
        [LArm, RArm] = compute_joint_positions(new_cart_position,previous_joints); % delta_x is the move size
        disp("Moving in direction x.")
        change=1;
    else
        change=0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    % y direction (left, right) - both arms together

    if abs(F_y) > lowest_F_value
        [LArm, RArm] = compute_joint_positions(new_cart_position,previous_joints);
        disp("Moving in direction y.")
        change=1;
    else
        change=0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z direction (up, down) - both arms together

    if abs(F_z) > lowest_F_value
        [LArm, RArm] = compute_joint_positions(new_cart_position,previous_joints);
        disp("Moving in direction z.")     
        change=1;
    else
        change=0;
    end

    
    % Add new joint positions to array joint_positions


%     new_joints.Position(1:7) = LArm;
%     new_joints.Position(8:14) = RArm;
    new_joints.Position(1:7) = LArm;
    new_joints.Position(8:14) = RArm;


    % checks if limit was reached and returns the name of the joint that
    % has reached the limit

    [stop_movement] = is_limit_reached(new_joints.Position,new_joints.Name);
    
   
%     if stop_movement == 1 && F_value < lowest_F_value
%         for i=1:3
%             fprintf("Trying to correct in direction %d.",i)
%             cart_position_test = new_cart_position;
%             for j=1:num_limbs
%                 cart_position_test(j,i) = cart_position_test(j,i) + 0.05;
%                 [LArm_test, RArm_test] = compute_joint_positions(cart_position_test,joint_positions);
%                 joints = joint_positions.Position;
%                 joints(1:7) = LArm_test;
%                 joints(8:14) = RArm_test;
%                 [is_limit] = is_limit_reached(joints,joint_positions.Name);
%                 if is_limit ~= 1
%                     LArm = LArm_test;
%                     RArm = RArm_test;
%                     stop_movement = 0;
%                 end
%             end
%         end
%     end

    % Change 7th joint position to initial value

    LArm(7) = initial_joint_position_arm_left_7_joint;
    RArm(7) = initial_joint_position_arm_right_7_joint;


    % Publish new joint positions

    joints_test = [LLeg' RLeg' LArm' RArm' Head' Torso'];
    all_joints = [all_joints; joints_test];

%     if all(abs(new_joints.Position-previous_joints.Position)<0.10) && stop_movement ~= 1 % how much can joints move in one step
    if stop_movement ~= 1

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

        current_cart_position = new_cart_position;
    else
        disp("Stopping this move.")
%         quasi_integral = all_q_integrals(end-1);
        new_cart_position = current_cart_position; % new_cart_positions are set to previous cartesic positions so that it doesn't save unused positions
        new_joints = previous_joints;
    end

    previous_joints = new_joints;

    all_cart_positions_L = [all_cart_positions_L; new_cart_position(1,:)];
    all_cart_positions_R = [all_cart_positions_R; new_cart_position(2,:)];

    time = r.TotalElapsedTime;
%     fprintf('Time Elapsed: %f\n',time)

    waitfor(r);


end

% Display changes in joint positions that occured during the program run

% disp("RArm")
% 
% disp("LArm")
% 
% disp("Torso")

disp(all_read_joints)
disp(initial_joints)
% disp(all_F_values)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTIONS

function [stop] = inside_work_zone(cart_position)

    global x_up_limit
    global x_low_limit
    global y_up_limit
    global y_low_limit
    global z_up_limit
    global z_low_limit

    global num_limbs

    for i=1:num_limbs
        if ~all(cart_position(i,1:3) < [x_up_limit,y_up_limit(i),z_up_limit])
            fprintf("Upper limit of work space reached.")
            stop = 1;
        elseif ~all(cart_position(i,1:3) > [x_low_limit,y_low_limit(i),z_low_limit])
            fprintf("Lower limit of work space reached.")
            stop = 1;
        else
            stop = 0;
        end
    end
end

function [stop_movement] = is_limit_reached(joint_positions, joint_names)

    global new_upper_limit;
    global new_lower_limit;


    if all(joint_positions < new_upper_limit) & all(joint_positions > new_lower_limit)
        disp('all joints within limit')
        stop_movement = 0;
    else
        for i=1:32
            if joint_positions(i) > new_upper_limit(i)
                fprintf('Joint: %s (index %d) has reached 90%% of upper limit\n',string(joint_names(i)),i);
                stop_movement = 1;
            end
            if joint_positions(i) < new_lower_limit(i)
                fprintf('Joint: %s (index %d) has reached 90%% of lower limit\n',string(joint_names(i)),i);
                stop_movement = 1;
            end
        end
    end
end
    

function [LArm, RArm] = compute_joint_positions(new_cart_position,joint_position) % direction tells in which direction (x,y,z) gripper should move
                                                                            % to move in x direction (forward, back) the value is 1
                                                                            % to move in y direction (left, right) the value is 2
                                                                            % to move in z direction (up, down) the value is 3

    global d_ik;

    cart_position = new_cart_position;

%     all_joint_positions = initial_joints.Position;
    
%     LArm = all_joint_positions(1:7);
%     RArm = all_joint_positions(8:14);
%     
%     LLeg = all_joint_positions(19:24);
%     RLeg = all_joint_positions(25:30);
%     
%     Head = all_joint_positions(17:18);
%     Torso = all_joint_positions(31:32);
    
    % Read joint positions only for left arm
    
    left_arm_joint_positions = joint_position.Position(1:7);
    left_arm_joint_names = joint_position.Name(1:7);
    
    right_arm_joint_positions = joint_position.Position(8:14);
    right_arm_joint_names = joint_position.Name(8:14);

    
    %%%%%%%%%%%%%%%%%%
    % Inverse kinematics
    %%%%%%%%%%%%%%%%%%
    
    ik_client = rossvcclient("/compute_ik",'moveit_msgs/GetPositionIK');
    
    ik_req_LArm = rosmessage(ik_client);
    
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

%     joint_constraint = rosmessage("moveit_msgs/JointConstraint");
%     joint_constraint.JointName = 'arm_left_7_joint';
%     joint_constraint.Position = left_arm_joint_positions(7);
%     joint_constraint.ToleranceAbove = 0.01;
%     joint_constraint.ToleranceBelow = 0.01;
%     joint_constraint.Weight = 0.5;
%     ik_req_LArm.IkRequest.Constraints.JointConstraints = joint_constraint;

%     ik_req_LArm.IkRequest.Constraints.JointConstraints.JointName = 'arm_left_7_joint';
%     ik_req_LArm.IkRequest.Constraints.JointConstraints.Position = 0.008;
    
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
        error_name = d_ik(string(ik_resp_RArm.ErrorCode.Val));
        fprintf("FK error, value: %d, name: %s\n",ik_resp_RArm.ErrorCode.Val,error_name)
    end
    
    LArm = ik_resp_LArm.Solution.JointState.Position(15:21);
    RArm = ik_resp_RArm.Solution.JointState.Position(29:35);
end
