L = Simulink.Parameter;
L.Value = 0.2; % Wheel base default value
L.CoderInfo.StorageClass = 'ExportedGlobal';

Ts_mpc = Simulink.Parameter;
Ts_mpc.Value = 0.02;


%%
%Init for simulation
pose_s = struct('ActorID', 0, 'Position', [0,0,0], 'Velocity', [0,0,0], ...
            'Roll', 0, 'Pitch', 0, 'Yaw', 0, 'AngularVelocity', [0,0,0]);

Simulink.Bus.createObject(pose_s);

