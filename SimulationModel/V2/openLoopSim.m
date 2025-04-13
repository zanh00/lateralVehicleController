ss_gain = cloffset(mpc3);
disp(ss_gain);

%%
T = 1500;
r = [0 0;
     0.5 0];
sim(mpc1, T, r);