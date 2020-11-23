classdef lidarPlanner
    %LIDARPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
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
            obj.Planner.MaxIterations=1e6;
            obj.Planner.MaxNumTreeNodes=1e6;
            obj.Planner.MaxConnectionDistance = 1;
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

