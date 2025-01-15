clear

load("mdlCLO3tf.mat")
Ts = G.Ts;

%% Plant Model
figure(1)
rlocus(G)
axis equal

figure(2)
bode(G)

%% Noise Model
zH = zero(H);
pH = pole(H);

figure(3)
rlocus(H,1)
axis equal

figure(4)
bode(H)

% %% Lead Compensator (F1)
% zF1 = 1;  % cancel pole at 1
% pF1 = 0.1;
% F1 = zpk(zF1,[pF1 0],1,Ts);  % pole at 0 for delay in output
% 
% F = F1;
% 
% figure(5)
% bode(G*F)
% 
% figure(6)
% rlocus(G*F)
% axis equal

% %% Controller Delay
% kF0 = 1;
% F0 = zpk([],0,kF0,Ts);
% F = F0;
F0 = 1;
F = F0;

%% Lead Compensator (F1)
zF1 = 0.95;
pF1 = 0.78;
kF1 = 1;
F1 = zpk(zF1,pF1,kF1,Ts);

omegaF1 = sqrt(log(zF1)*log(pF1))/Ts;
alphaF1 = (-log(pF1)/omegaF1/Ts)^2;

F = F0*F1;

figure(5)
bode(G*F)

figure(6)
rlocus(G*F)
axis equal

figure(4)
bode(H)
hold on
bode(1+G*F)
hold off

% %% Lag Compensator (F2)
% zF2 = 0.999;
% pF2 = 1;
% F2 = zpk(zF2,pF2,1,Ts);
% 
% F = F0*F1*F2;
% 
% figure(7)
% bode(G*F)
% 
% figure(4)
% bode(H)
% hold on
% bode(1+G*F)
% hold off

%% Lag Compensator (F2)
omegaF2 = 0.01;
alphaF2 = 1/100;
zF2 = exp(-omegaF2*Ts/sqrt(alphaF2));
pF2 = exp(-omegaF2*Ts*sqrt(alphaF2));
% zF2 = 0.7;
% pF2 = 0.9999;
F2 = zpk(zF2,pF2,1,Ts);

F = F0*F1*F2;

figure(7)
bode(G*F)

figure(8)
rlocus(G*F)
axis equal

figure(4)
bode(H)
hold on
bode(1+G*F)
hold off

%% Original Controller
numF = [1 -1.9 0.9];
denF = [1 -1.099 0.0999];
F = zpk(tf(numF,denF,Ts));

%% Controller to Cancel Noise Model
F = minreal(G^-1*(H-1));

%% Closed Loop with No Disturbance
Gcl = minreal(G*F/(1+G*F));
Su = minreal(F/(1+G*F));

tmax = 15;

figure(21)

subplot(2,2,1)
rlocus(Gcl,1)
axis equal

subplot(2,2,2)
step(Gcl,tmax)

subplot(2,2,3)
bode(Gcl)

subplot(2,2,4)
step(Su,tmax)

%% Simulate with Disturbance
% (actual white noise disturbance may correlate with reference)
amplitude = 30;
period = 200;  % integer
cycles = 2;

N = cycles*period;
t = (0:N-1)';

r = [amplitude; -amplitude];
r = repelem(r,period/2);
r = repmat(r,cycles,1);

e = sqrt(17)*randn(N,1);

%% Closed Loop
Gcl = minreal(G*F/(1+G*F));
Hcl = minreal(H/(1+G*F));
Su = minreal(F/(1+G*F));

ysim = lsim(Gcl,r);
y = ysim + lsim(Hcl,e);
ye = ysim + e;
usim = lsim(Su,r);
u = usim + lsim(-Su,e);

figure(101)
subplot(2,1,1)
plot([ysim y ye])
subplot(2,1,2)
plot([usim u])

% Prediction Filter (Kalman Filter)
Pfilter = [H^-1*G (1-H^-1)];
ypred = lsim(Pfilter,[u y]);

subplot(2,1,1)
hold on
plot(ypred)
hold off

%% Closed Loop with Disturbance Observer
% (actual white noise disturbance may correlate with reference)
Gcl = minreal(G*F/(1+G*F));
Hcl = minreal(1/(1+G*F));
Su = minreal(F/(1+G*F));

ysim = lsim(Gcl,r);
y = ysim + lsim(Hcl,e);
ye = ysim + e;
usim = lsim(Su,r);
u = lsim(H*F,r-y) - lsim(G^-1*(H-1),y);

figure(102)
subplot(2,1,1)
plot([ysim y ye])
subplot(2,1,2)
plot([usim u])

% Prediction Filter (Kalman Filter)
Pfilter = [H^-1*G (1-H^-1)];
ypred = lsim(Pfilter,[u y]);

subplot(2,1,1)
hold on
plot(ypred)
hold off

%% Closed Loop with Prediction Filter and Disturbance Observer
% (actual white noise disturbance may correlate with reference)
Gcl = minreal(G*F/(1+G*F));
% Hcl = 1;
Su = minreal(F/(1+G*F));

ysim = lsim(Gcl,r);
y = ysim + e;
ye = ysim + e;
usim = lsim(Su,r);
u = lsim(H*F/(1+G*F),r) - lsim((H-1)*(F+G^-1)/(1+G*F),y);

figure(103)
subplot(2,1,1)
plot([ysim y ye])
subplot(2,1,2)
plot([usim u])

% Prediction Filter (Kalman Filter)
Pfilter = [H^-1*G (1-H^-1)];
ypred = lsim(Pfilter,[u y]);

subplot(2,1,1)
hold on
plot(ypred)
hold off