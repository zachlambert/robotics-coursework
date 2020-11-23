classdef lidarPlanner
    properties
        GoalPose,
        SensorOffset,
        SensorAngles,
        MaxRange,
        Validator,
        Planner
    end
    
    methods
        function obj = lidarPlanner(goalPose, sensorOffset, sensorAngles, maxRange, mapRes, mapXLimits, mapYLimits)
            obj.GoalPose = goalPose;
            obj.SensorOffset = sensorOffset;
            obj.SensorAngles = sensorAngles;
            obj.MaxRange = maxRange;
            
            map = binaryOccupancyMap(...
                [mapXLimits(2)-mapXLimits(1), mapYLimits(2)-mapYLimits(1)],...
                mapRes...
            );
            
            ss = stateSpaceSE2;
            ss.StateBounds = [mapXLimits; mapYLimits; -pi pi];
            obj.Validator = validatorOccupancyMap(ss);
            obj.Validator.Map = map;

            obj.Planner = plannerRRTStar(ss, validator);
            % Only want a short-term plan, so don't do many iterations
            obj.Planner.MaxIterations=1000;
            obj.Planner.MaxNumTreeNodes=100;
            obj.Planner.MaxConnectionDistance = 0.8;
        end
        
        function update(obj, pose, lidar_range)
            scan = lidarScan(lidar_range, obj.SensorAngles);
            insertRay(obj.Validator.Map, pose, scan, obj.maxRange);
        end
        
        function waypoints = plan(obj, pose)
            obj.Planner.StateValidator = obj.Validator;
            [refpath, ~] = plan(obj.Planner, pose, obj.GoalPose);
            waypoints = refpath.States(:,1:2);
        end
    end
end

