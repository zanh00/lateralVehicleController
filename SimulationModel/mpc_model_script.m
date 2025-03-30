%% create MPC controller object with sample time
mpc3 = mpc(plant_model, Ts_mpc.Value);
%% specify prediction horizon
mpc3.PredictionHorizon = 20;
%% specify control horizon
mpc3.ControlHorizon = 3;
%% specify nominal values for inputs and outputs
%mpc3.Model.Nominal.U = 0;
mpc3.Model.Nominal.Y = [0;0];
%% specify scale factors for inputs and outputs
mpc3.MV(1).ScaleFactor = 0.1;
mpc3.OV(1).ScaleFactor = 0.5;
mpc3.OV(2).ScaleFactor = 10;
%% specify constraints for MV and MV Rate
mpc3.MV(1).Min = -0.20;
mpc3.MV(1).Max = 0.20;
%% specify weights
mpc3.Weights.MV = 0;
mpc3.Weights.MVRate = 0.1;
mpc3.Weights.OV = [4 15];
mpc3.Weights.ECR = 1000;

disp(mpc3.PredictionHorizon)
disp(mpc3.ControlHorizon)
disp(mpc3.Weights)

