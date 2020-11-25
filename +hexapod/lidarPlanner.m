classdef lidarPlanner < matlab.System
    % Public, tunable
    properties
        GoalPose,
        SensorAngles,
        MapRes,
        MapXLimits,
        MapYLimits,
    end
    
    properties(DiscreteState)
        Map,
        Validator,
        Planner,
        PathUpdateCounter
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            
        end
        
        function waypoints = stepImpl(obj, pose, lidar_range)
            scan = lidarScan(lidar_range, obj.SensorAngles);
            insertRay(obj.Map, pose, scan, obj.maxRange);
               
            % TODO: Make the following only occur when the
            % current path is invalidated
            obj.PathUpdateCounter = obj.PathUpdateCounter + 1;
            if obj.PathUpdateCounter > 20
                obj.PathUpdateCounter = 0;
                obj.Validator.Map = obj.Map;
                obj.Validator.Map.inflate(1.2/res);
                obj.Planner.StateValidator = obj.Validator;
                [refpath, ~] = plan(obj.Planner, pose, obj.GoalPose);
                waypoints = refpath.States(:,1:2);
            end
        end
        
        function resetImpl(obj)
            obj.Map = binaryOccupancyMap(...
                [obj.MapXLimits(2)-obj.MapXLimits(1), obj.MapYLimits(2)-obj.MapYLimits(1)],...
                obj.MapRes...
            );
            
            ss = stateSpaceSE2;
            ss.StateBounds = [mapXLimits; mapYLimits; -pi pi];
            obj.Validator = validatorOccupancyMap(ss);
            
            obj.Planner = plannerRRTStar(ss, obj.Validator);
            % Only want a short-term plan, so don't do many iterations
            obj.Planner.MaxIterations=1000;
            obj.Planner.MaxNumTreeNodes=100;
            obj.Planner.MaxConnectionDistance = 0.8;
            
            obj.PathUpdateCounter = 0;
        end
    end
end

