classdef Trajectory
% Trajecotry.m     e.anderlini@ucl.ac.uk     09/10/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This class generates trajectory for the UUV in 3D space and time.
% At the moment, the following trajectories are supported:
% * fixed point in state (hovering);
% * straight line;
% * helix;
% * minimum-snap trajectory through specified waypoints.
% These curves have been taken from research in quadcopters. In particular,
% the robotics course from the University of Pennsylvania has been used for
% inspiration. For reference, see N. Michael, D. Mellinger, Q. Lindsey and
% V. Kumar (2010). Experimental evaluation of multirobot aerial control
% algorithms. IEEE Robotics & Automation Magazine, v. 17, September, pp.
% 56-65.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Public properties:
    properties
        des_pos
        des_vel
        des_acc
        duration
        trjtype
        waypoints
    end
    
    %% Protected properties:
    properties (Access = protected)
        % Minimum snap:
        alpha
        J
        K
        m
        n
        S
        T
        X
        Y
    end
    
    %% Public methods:
    methods
        %% Initialization function:
        function obj = Trajectory(trj,waypoints,duration)  
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Input:
            % t: time series;
            % x: states matrix.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if nargin==3
                obj.trjtype = trj;
                obj.waypoints = waypoints;
                obj.duration = duration;
                % Initialize the trajectory generation functions:
%                 if strcmp(trj,'line')
%                     obj = obj.initialize_line();
%                 elseif strcmp(trj,'helix')
%                     obj = obj.initialize_helix();
                if strcmp(trj,'minimum_snap')
                    obj = obj.initialize_minimum_snap;
                else
                    error(['Only point, line, helix and minimum snap',...
                        'trajectories supported']);
                end
                % Initialize the desired output variables:
                obj.des_pos = zeros(3,1);
                obj.des_vel = zeros(3,1);
                obj.des_acc = zeros(3,1);
            else
                error('Please specify trajectory type and waypoints');
            end
        end
        
        %% Real-time trajectory generation:
        function obj = trajectory_generation(obj,t)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Input:
            % t: current time.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initialize the trajectory generation functions:          
%             if strcmp(obj.trjtype,'line')
%                 obj.line(obj,t);
%             elseif strcmp(obj.trjtype,'helix')
%                 obj.helix(obj,t);
            if strcmp(obj.trjtype,'minimum_snap')
                obj = obj.minimum_snap(t);
            else
                obj = obj.fixed_point();
            end
        end
    end
   
    %% Protected methods:
    methods (Access = protected)
        %% Initialization function for the straight line trajectory:
%         function obj = initialize_line(obj)
%             this;
%         end
        
        %% Initialization function for the minimum-snap trajectory:
        function obj = initialize_minimum_snap(obj)
            % Get number of waypoints:
            obj.m = size(obj.waypoints,1);
            % Get number of segments:
            obj.n = obj.m-1;
            % Get the length of all segments:
            l = zeros(obj.n,1);
            for i=1:obj.n
                l(i) = sqrt(sum((obj.waypoints(i+1,:)-...
                    obj.waypoints(i,:)).^2));
            end
            % Express the length as a fraction of the total length:
            lp = l/sum(l);
            % Get the duration of each segment:
            obj.T = lp*obj.duration;
            % Get the start point of each segment:
            obj.S = zeros(obj.m,1);
            for i=2:obj.m
                obj.S(i) = obj.S(i-1)+obj.T(i-1);
            end

            % Initialize matrix A, vector b and vector alpha:
            alpha = zeros(8*obj.n,3);          % 3-D
            A = zeros(8*obj.n);
            b = zeros(8*obj.n,3);              % 3-D
            % Fill-in vector b:
            for i=1:obj.n
                j = 4+8*(i-1);                 % index
                b(j:j+1,:) = obj.waypoints(i:i+1,:);
            end
            % Prepare matrices for completion of matrix A:
            C = zeros(1,8); C(1) = 1;
            D = zeros(1,8); D(2) = 1;
            E = zeros(1,8); E(3) = 1;
            F = zeros(1,8); F(4) = 1;
            G = zeros(1,8); G(5) = 1;
            H = zeros(1,8); H(6) = 1;
            I = zeros(1,8); I(7) = 1;
            obj.J = [0,1,2,3,4,5,6,7];
            obj.K = [0,0,1,3,6,10,15,21];
            L = [0,0,0,1,4,10,20,35];
            M = [0,0,0,0,1,5,15,35];
            N = [0,0,0,0,0,1,6,21];
            O = [0,0,0,0,0,0,1,7];
            P = [C;ones(1,8)];
            Q = [D;E;F];
            R = [obj.J;obj.K;L];
            U = [R;M;N;O];
            V = -[Q;G;H;I];
            obj.X = [0,0,1,2,3,4,5,6];
            obj.Y = [0,0,0,1,2,3,4,5];
            % Fill-in matrix A:
            A(1:3,1:8) = Q;
            for i=1:obj.n-1
                j = 1+8*(i-1);         % index
                k = j+3;               % index
                A(k:k+1,j:j+7) = P;
                A(k+2:k+7,j:j+7) = U;
                A(k+2:k+7,j+8:j+15) = V;
            end
            A(end-4:end-3,end-7:end) = P;
            A(end-2:end,end-7:end) = R;
            % Obtain the vectors of coefficients:
            for i=1:3
                alpha(:,i) = A \ b(:,i);
            end
            obj.alpha = alpha';
        end
        
        %% Fixed-point trajectory generator:
        function obj = fixed_point(obj)
            obj.des_pos = obj.waypoints(1,:)';
            obj.des_vel = zeros(3,1);
            obj.des_acc = zeros(3,1);
        end
        
        %% Line trajectory generator:
%         function obj = line(obj,t)
%             this;
%         end
        
        %% Minimum-snap trajectory generator:
        function obj = minimum_snap(obj,t)
            if t<obj.S(obj.m)
                % Identify the current segment:
                for i=1:obj.n
                   if t<obj.S(i+1)
                       break;
                   end
                end
                % Calculate the desired trajectory:
                tmp = (t-obj.S(i))/obj.T(i);
                for j=1:3
                    k = 1+8*(i-1);
                    obj.des_pos(j) = sum(obj.alpha(j,k:k+7).*tmp.^obj.J);
                    obj.des_vel(j) = sum(obj.J.*obj.alpha(j,k:k+7).*...
                        tmp.^obj.X);
                    obj.des_acc(j) = sum(obj.K.*obj.alpha(j,k:k+7).*...
                        tmp.^obj.Y);
                end
            else
                obj.des_pos = obj.waypoints(obj.m,:)';
                obj.des_vel = zeros(3,1);
                obj.des_acc = zeros(3,1);
            end
        end
    end
end