v = 2;
L = 0.35;
Ts = 0.02;

A3 =[0 0 0 0
     0 0  v v;
     0 0  0 v/L;
     0 0 0  0];

B3 = [0
      0;
      0;
      1];

C3 = [0 1 0 0;
      0 0 1 0];
D3 = [0;
      0];

% A3 =[0  v v;
%      0  0 v/l;
%      0 0  0];
% 
% B3 = [0
%       0;
%       1];
% 
% C3 = [1 0 0;
%       0 1 0];
% D3 = [0;
%       0];

%plant_model_noMD = ss(A3, B3, C3, D3, Ts);
plant_model_noMD = ss(A3, B3, C3, D3, 0);

plant_model_noMD.InputName = {'Steering rate'};
plant_model_noMD.OutputName = {'Lateral position', 'Yaw angle'};

plant_model_noMD=setmpcsignals(plant_model_noMD, 'MV', 1, 'MO', [1 2]);

plant_discrete = c2d(plant_model_noMD, Ts, 'zoh');

%%
%Init for simulation
pose_s = struct('ActorID', 0, 'Position', [0,0,0], 'Velocity', [0,0,0], ...
            'Roll', 0, 'Pitch', 0, 'Yaw', 0, 'AngularVelocity', [0,0,0]);

Simulink.Bus.createObject(pose_s);