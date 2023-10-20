classdef talos_arm_spec < robot
    %Talos arm specifications
    %
  
    methods (Access=protected)
        function Specifications(robot)
            robot.nj=7;
            robot.q_home= [0 0.8 -0.7 -0.5 0.8 -0.33 0.37]'; % home joint configuration
            robot.q_max=[0.785398163397 2.87106661953 2.42600766027 -0.00349065850399 2.51327412287 1.37008346282 0.680678408278]';   % upper joint limits
            robot.q_min=[-1.57079632679 0.00872664625997 -2.42600766027 -2.23402144255 -2.51327412287 -1.37008346282 -0.680678408278]';  % lower joint limits
            robot.qdot_max=[2.1750 2.1750 2.1750 2.1750 2.6100 2.6100 2.6100]'; % maximal joint velocities
            robot.v_max=[1.5 1.5 1.5 2 2 2]'; % maximal task velocities
        end
    end
    
    methods
        %------------------------------------------------------------------
        % Kinematic model methods
        %
        
        function [p,R,J]=Kinmodel(robot,q)
            %KINMODEL Kinematic model
            %
            % Usage:
            %    [p,R,J]=Kinmodel(q)
            %
            % Input:
            %   q   joint position (nj x 1)
            %
            % Output:
            %   p   task position [x y z] (3 x 1)
            %   R   rotational matrix (3 x 3)
            %   J   Jacobian matrix (6 x nj)
            %
            
            if nargin==1
                q=robot.actual_int.q;
            end
            [p,R,J]=kinmodel_talos_arm(q,robot.TCP);
        end
    end % methods
end