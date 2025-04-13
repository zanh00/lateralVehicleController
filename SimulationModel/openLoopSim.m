ss_gain = cloffset(mpc3);
disp(ss_gain);

%%
T = 150;
r = [0 0;
     1 0];
sim(mpc1_1, T, r);