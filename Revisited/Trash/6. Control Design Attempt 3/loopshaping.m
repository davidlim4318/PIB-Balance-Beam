clear

load("mdlCLO4tf.mat")
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

% figure(3)
% rlocus(H,1)
% axis equal

figure(4)
bode(H)

F0 = 1;
F = F0;

%% Lead Compensator (F1)
zF1 = 0.98;
pF1 = 0.2;
kF1 = 1;

omegaF1 = sqrt(log(zF1)*log(pF1))/Ts;
alphaF1 = (-log(pF1)/omegaF1/Ts)^2;

% omegaF1 = 3;
% alphaF1 = 20;
% zF1 = exp(-omegaF1*Ts/sqrt(alphaF1));
% pF1 = exp(-omegaF1*Ts*sqrt(alphaF1));
% kF1 = 1;

F1 = zpk(zF1,pF1,kF1,Ts);

F = F0*F1;

figure(5)
bode(G*F)

figure(6)
rlocus(G*F)
axis equal

figure(4)
bode(H)
hold on
bode(1+G*F1)
hold off

%% Notch Filter (F2)
% omegaF2 = 6;
zF2 = exp(-1*1j*omegaF2*Ts);
pF2 = exp(-omegaF2*Ts);

F2 = zpk([zF2 conj(zF2)],[pF2 pF2],1,Ts);

F = F0*F1*F2;

figure(5)
bode(G*F1)
hold on
bode(G*F)
hold off

figure(6)
rlocus(G*F)
axis equal

figure(4)
bode(H)
hold on
bode(1+G*F1)
bode(1+G*F)
hold off

% %% Lag Compensator (F2)
% omegaF2 = 0.01;
% alphaF2 = 1/100;
% zF2 = exp(-omegaF2*Ts/sqrt(alphaF2));
% pF2 = exp(-omegaF2*Ts*sqrt(alphaF2));
% 
% F2 = zpk(zF2,pF2,1,Ts);
% 
% F = F0*F1*F2^2;
% 
% figure(5)
% bode(G*F1)
% hold on
% bode(G*F)
% hold off
% 
% figure(6)
% rlocus(G*F)
% axis equal
% 
% figure(4)
% bode(H)
% hold on
% bode(1+G*F1)
% bode(1+G*F)
% hold off

%% Adjust Gain
omega = 3;
K = 1/abs(freqresp(G*F,omega));

F = K*F;

figure(5)
hold on
bode(G*F)
hold off

%% Original Controller
zF = 0.97;
pF = 0.5;
kF = 1;
F = zpk(zF,pF,kF,Ts);

% %% Controller to Cancel Noise Model
% F = minreal(G^-1*(H-1));

%% Closed Loop with No Disturbance
Gcl = minreal(G*F/(1+G*F));
Su = minreal(F/(1+G*F));

tmax = 5;

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
period = 600;  % integer
cycles = 2;

N = cycles*period;
t = (0:N-1)';

r = [amplitude; -amplitude];
r = repelem(r,period/2);
r = repmat(r,cycles,1);

e = sqrt(13)*randn(N,1);
% load("e.mat")
% e = e(1:N);

%% Closed Loop
Gcl = minreal(G*F/(1+G*F));
Hcl = minreal(H/(1+G*F));
Su = minreal(F/(1+G*F));

ysim = lsim(Gcl,r);
y = ysim + lsim(Hcl,e);
ye = ysim + e;
usim = lsim(Su,r);
u = usim + lsim(-Su,e);

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

%% Closed Loop with 1-Step Prediction Filter
Gcl = minreal(G*F/(1+G*F));
Hcl = minreal((H+G*F)/(1+G*F));
Su = minreal(F/(1+G*F));

ysim = lsim(Gcl,r);
y = ysim + lsim(Hcl,e);
ye = ysim + e;
usim = lsim(Su,r);
u = lsim(F/(1+H^-1*G*F),r) - lsim(F*(1-H^-1)/(1+H^-1*G*F),y);

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

%% Closed Loop with 1-Step Prediction Filter and Filtered Disturbance Observer
T = 0.1;
L = tf(Ts/T,[1 Ts/T-1],Ts);

Gcl = minreal(G*F/(1+G*F));
Hcl = minreal((H+G*F+L-L*H)/(1+G*F));
Su = minreal(F/(1+G*F));

ysim = lsim(Gcl,r);
y = ysim + lsim(Hcl,e);
ye = ysim + e;
usim = lsim(Su,r);
u = lsim(F/(1-L*(1-H^-1)+H^-1*G*F),r) - lsim((F+L*G^-1)*(1-H^-1)/(1-L*(1-H^-1)+H^-1*G*F),y);

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

%% Closed Loop with 2-Step Prediction Filter and Disturbance Observer
% (actual white noise disturbance may correlate with reference)
[numH,denH] = tfdata(H);
h1 = numH{1}(2) - denH{1}(2);
D = tf([1 h1],[1 0],Ts);

Gcl = minreal(G*F/(1+G*F));
Hcl = D;
Su = minreal(F/(1+G*F));

ysim = lsim(Gcl,r);
y = ysim + lsim(Hcl,e);
ye = ysim + e;
usim = lsim(Su,r);
u = lsim(D^-1*H*F/(1+G*F),r) - lsim((D^-1*H-1)*(F+G^-1)/(1+G*F),y);

figure(104)
subplot(2,1,1)
plot([ysim y ye])
subplot(2,1,2)
plot([usim u])

% Prediction Filter (Kalman Filter)
Pfilter = [D*H^-1*G (1-D*H^-1)];
ypred = lsim(Pfilter,[u y]);

subplot(2,1,1)
hold on
plot(ypred)
hold off

%% Closed Loop with 2-Step Prediction Filter and Filtered Disturbance Observer
% (actual white noise disturbance may correlate with reference)
T = 0.05;
[numL,denL] = butter(2,Ts/T/pi);
L = tf(numL,denL,Ts);

[numH,denH] = tfdata(H);
h1 = numH{1}(2) - denH{1}(2);
D = tf([1 h1],[1 0],Ts);

Gcl = minreal(G*F/(1+G*F));
Hcl = minreal((H+D*G*F+L*D-L*H)/(1+G*F));
Su = minreal(F/(1+G*F));

ysim = lsim(Gcl,r);
y = ysim + lsim(Hcl,e);
ye = ysim + e;
usim = lsim(Su,r);
u = lsim(F/(1-L*(1-D*H^-1)+D*H^-1*G*F),r) - lsim((F+L*G^-1)*(1-D*H^-1)/(1-L*(1-D*H^-1)+D*H^-1*G*F),y);

figure(105)
subplot(2,1,1)
plot([ysim y ye])
subplot(2,1,2)
plot([usim u])

% Prediction Filter (Kalman Filter)
Pfilter = [D*H^-1*G (1-D*H^-1)];
ypred = lsim(Pfilter,[u y]);

subplot(2,1,1)
hold on
plot(ypred)
hold off