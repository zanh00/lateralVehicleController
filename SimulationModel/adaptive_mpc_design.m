v = 10;
L = 2;

A2 = [0 0 -v 0;
     0 0  v v;
     0 0  0 v/L;
     0 0  0  0];

B2 = [0 0;
     1 0;
     0 0;
     0 1];

C2 = [0 1 0 0;
      0 0 1 0];
D2 = [0 0;
     0 0];

plant_model = ss(A2, B2, C2, D2, 0.05);
plant_model.InputName = {'Yaw rate', 'Steering rate'};
plant_model.OutputName = {'Lateral position', 'Yaw angle'};

plant_model=setmpcsignals(plant_model, 'MV', 2, 'MO', [1 2], 'MD', 1);
