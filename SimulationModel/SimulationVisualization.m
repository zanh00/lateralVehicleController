
% Scenario file that simulation was run on
simulated_scenario = "C:\Users\zanhe\Documents\lateralVehicleController\SimulationModel\driving_scenarios\agressive_turn.mat";
load(simulated_scenario);

% Extract road senters from the senario file
road_centers = data.RoadSpecifications.Centers;

% Create the drivingScenario object and ego car
velocity = double(out.logsout.get("velocity_sim").Values.Data);
[scenario, egoVehicle] = createDrivingScenario(out.egoPose, velocity, road_centers);

% ?? This sensor code is probably unnecesary since we don't visualize
% ?? detection her

% Create all the sensors
sensor = createSensor(scenario);

% Add sensor to scenario
addSensors(scenario, sensor, egoVehicle.ActorID);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {}, 'INSMeasurements', {});
time  = scenario.SimulationTime;

if ~isa(sensor,'insSensor')
    poses = targetPoses(scenario,sensor.SensorIndex);
end
% Generate the ego vehicle lane boundaries
if isa(sensor, 'visionDetectionGenerator')
    maxLaneDetectionRange = min(500,sensor.MaxRange);
    lanes = laneBoundaries(egoVehicle, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
end
% Generate detections for the sensor
objectDetections = {};
ptClouds = [];
insMeas = [];
[laneDetections, ~, isValidLaneTime] = sensor(lanes, time);

% Aggregate all detections into a structure for later use
if isValidLaneTime
    allData(end + 1) = struct( ...
        'Time',       scenario.SimulationTime, ...
        'ActorPoses', actorPoses(scenario), ...
        'ObjectDetections', {objectDetections}, ...
        'LaneDetections', {laneDetections}, ...
        'PointClouds',   {ptClouds}, ...
        'INSMeasurements',   {insMeas});
end

%%
hFigure = figure;
hFigure.Position(3) = 900;

hPanel1 = uipanel(hFigure,'Units','Normalized','Position',[0 1/4 1/2 3/4],'Title','Scenario Plot');
hPanel2 = uipanel(hFigure,'Units','Normalized','Position',[0 0 1/2 1/4],'Title','Chase Plot');
hPanel3 = uipanel(hFigure,'Units','Normalized','Position',[1/2 0 1/2 1],'Title','Bird''s-Eye Plot');

hAxes1 = axes('Parent',hPanel1);
hAxes2 = axes('Parent',hPanel2);
hAxes3 = axes('Parent',hPanel3);

%%
% assign scenario plot to first axes and add indicators for ActorIDs 1 and 2
plot(scenario, 'Parent', hAxes1,'ActorIndicators',1);

% assign chase plot to second axes
chasePlot(egoVehicle, 'Parent', hAxes2);

% assign bird's-eye plot to third axes
egoCarBEP = birdsEyePlot('Parent',hAxes3,'XLimits',[-20 20],'YLimits',[-14 14]);
%fastTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','red','DisplayName','target','VelocityScaling',.5);
egoTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','blue','DisplayName','ego','VelocityScaling',.5, 'HistoryDepth', 20);
egoLanePlotter = laneBoundaryPlotter(egoCarBEP);
plotTrack(egoTrackPlotter, [0 0]);
egoOutlinePlotter = outlinePlotter(egoCarBEP);
%%
restart(scenario)
scenario.SampleTime = 0.04;
while advance(scenario)
    t = actorPoses(scenario);
    plotTrack(egoTrackPlotter, t.Position, t.Velocity);
    rbs = roadBoundaries(egoVehicle);
    plotLaneBoundary(egoLanePlotter, rbs);
    %[position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    %plotOutline(egoOutlinePlotter, position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);
    pause(0.02);
end

%%
plot(scenario, 'Waypoints','on');


% Release the sensor object so it can be used again.
release(sensor);

%%

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function sensor = createSensor(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensor = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [0.15 0], ...
    'Height', 0.2, ...
    'Pitch', -20, ...
    'MaxRange', 5, ...
    'DetectorOutput', 'Lanes only', ...
    'Intrinsics', cameraIntrinsics([800 799.999999999999],[320 240],[480 900]), ...
    'ActorProfiles', profiles);

end

function [scenario, egoVehicle] = createDrivingScenario(dataset, velocity, roadCenters)
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36], 'Width', 0.05)
    laneMarking('Solid', 'Width', 0.05)];
laneSpecification = lanespec(1, 'Width', 0.5, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 0.45, ...
    'Width', 0.15, ...
    'Height', 0.2, ...
    'Position', [0 0 0.01], ...
    'Yaw', 12, ...
    'RearOverhang', 0.1, ...
    'FrontOverhang', 0.1, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');

egopath_set = dataset.get("<Position>");
xy = egopath_set.Values.Data(1,1:2,:);
waypoints = squeeze(permute(xy, [3,1,2]));
smoothTrajectory(egoVehicle, waypoints, velocity);

end