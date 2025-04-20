v = 2;
L = 0.35;
Ts = 0.02;

A3 =[0 0 0 0
     0 0  v v;
     0 0  0 v/L;
     0 0 0  0];

% B3 = [0
%       0;
%       0;
%       1];

B3 = [0 0
      0 0;
      0 -1;
      1 0];

C3 = [0 1 0 0;
      0 0 1 0;
      0 0 0 1];
D3 = [0 0;
      0 0;
      0 0];

%plant_model_noMD = ss(A3, B3, C3, D3, Ts);
plant_model = ss(A3, B3, C3, D3, 0);

plant_model.InputName = {'Steering rate', 'Ref Yaw Rate'};
plant_model.OutputName = {'Lateral position', 'Yaw angle', 'Steering angle'};

plant_model=setmpcsignals(plant_model, 'MV', 1, 'MO', [1 3], 'MD', 2);

plant_discrete = c2d(plant_model, Ts, 'zoh');

%%
%Init for simulation
pose_s = struct('ActorID', 0, 'Position', [0,0,0], 'Velocity', [0,0,0], ...
            'Roll', 0, 'Pitch', 0, 'Yaw', 0, 'AngularVelocity', [0,0,0]);

Simulink.Bus.createObject(pose_s);