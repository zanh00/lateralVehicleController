%% create MPC controller object with sample time
mpc1 = mpc(plant_discrete, 0.02);
mpc1.Model.Plant=minreal(mpc1.Model.Plant);
%% specify prediction horizon
mpc1.PredictionHorizon = 20;
%% specify control horizon
mpc1.ControlHorizon = 3;
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = 0;
mpc1.Model.Nominal.Y = [0;0];
%%
mpc1.OV(1).ScaleFactor = 0.05;
mpc1.OV(2).ScaleFactor = 2.5;
mpc1.MV(1).ScaleFactor = 1;
mpc1.OV(3).ScaleFactor = 1;
mpc1.DV(1).ScaleFactor = 15;
%mpc1.OV(1).Min = -0.3;
%mpc1.OV(1).Max = 0.3;
%% specify weights
mpc1.Weights.MV = 0;
mpc1.Weights.MVRate = 0.1;
mpc1.Weights.OV = [2 6 0];
mpc1.Weights.ECR = 100000;

%% 
mpc1.OV(3).Min = -30;
mpc1.OV(3).Max = 30;
mpc1.MV(1).Min = -15;
mpc1.MV(1).Max = 15;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
%sim(mpc1, 501, mpc1_RefSignal, mpc1_MDSignal, options);
