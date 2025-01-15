clear

txt1 = readmatrix("expCL1.txt");
N = 1200;
t = txt1(1:N,1)/1000000;
r = txt1(1:N,2) - 135;
y = txt1(1:N,3) - 135;

Ts = mean(diff(t));

%% Input Signal
figure(1)
tiledlayout(2,2)
nexttile(1)
plot(r)

R = fft(r);
Phir = abs(R).^2/N;
w = linspace(0,pi,N/2+1);

nexttile(2)
plot(w,Phir(1:N/2+1))

%% Output Signal
nexttile(3)
plot(y)

Y = fft(y);
Phiy = abs(Y).^2/N;
w = linspace(0,pi,N/2+1);

nexttile(4)
plot(w,Phiy(1:N/2+1))

%% FIR LS
n = 30;  % even number
m = n+1;
idx = (m:N);
PHI = zeros(N-m+1,m);
for k = 1:m
    PHI(:,k) = r(idx+1-k);
end
Y = y(idx);
theta_FIR = PHI\Y;

mdl = fitlm(PHI,Y,'Intercept',false);
ci = coefCI(mdl,0.01);

k = 0:n;

%% Plot FIR Parameters
figure(5)
fill([k flip(k)],[ci(:,1);flip(ci(:,2))]/Ts,'b','FaceAlpha',0.2,'LineStyle','none')
hold on
plot(k,theta_FIR/Ts,'b-');
hold off

%% Hankel Matrix
N1 = (m+1)/2;  % plot depends on N1
H = hankel(theta_FIR(2:N1+1),theta_FIR(N1+1:2*N1-1));
[U,S,V] = svd(H);
sigma = diag(S);

p = min(N1-1,30);
figure(11)
tiledlayout(2,1)
nexttile(1)
plot(sigma(1:p),'r*');
nexttile(2)
semilogy(sigma(1:p),'r*');

%% Load Datasets
data0 = iddata(y,r,Ts);
data1 = iddata(txt1(N+1:end,3)-135,txt1(N+1:end,2)-135,Ts);
txt2 = readmatrix("expCL2.txt");
data2 = iddata(txt2(1:400,3)-130,txt2(1:400,2)-130,Ts);

%% Preliminary Parameter Estimation with ARMAX
mdltest = armax(data0,[3 3 3 1]);  % determine ideal model order using general model

% mdltest = bj(data0,[3 0 1 3 1]);  % determine ideal model order using general model

figure(21)
subplot(2,1,1);
compare(data0,mdltest,1);
subplot(2,1,2);
resid(data0,mdltest)

%% Validation Data 1: remaining data
figure(22)
subplot(2,1,1);
compare(data1,mdltest,1);
subplot(2,1,2);
resid(data1,mdltest)

%% Validation Data 2: new dataset
figure(23)
subplot(2,1,1);
compare(data2,mdltest,1);
subplot(2,1,2);
resid(data2,mdltest)

%% Use for Initial Parameters (guess)
roots(mdltest.B)
roots(mdltest.A)
mdltest.B

%% Parameter Estimation Using Defined Grey-Box Model
par = [0.5 0.95 0.02];  % canceled pole at 1
zF = 0.9;  % canceled zero at 1
pF = [0.1 0.999];
kF = 1;
aux = [zF pF kF];
mdlstruct = idgrey('dynamicsCLO3',par,'d',aux,Ts);
mdl = greyest(data0,mdlstruct,greyestOptions('DisturbanceModel','estimate'));

% par = [1 0.99 0.01 0.2 1];  % canceled pole at 1
% zF = 0.9;  % canceled zero at 1
% pF = [0.1 0.999];
% kF = 1;
% aux = [zF pF kF];
% mdlstruct = idgrey('dynamicsCLO3_2',par,'d',aux,Ts);
% mdl = greyest(data0,mdlstruct,greyestOptions('DisturbanceModel','model'));

figure(31)
subplot(2,1,1);
compare(data0,mdl,1);
subplot(2,1,2);
resid(data0,mdl)

% par = [1.5 1.5 0.8 0.1 0.01];  % canceled pole at 1
% zF = 0.9;  % canceled zero at 1
% pF = [0.1 0.999];
% kF = 1;
% aux = [zF pF kF]';
% mdlstruct = idgrey('dynamicsCLO4',par,'d',aux,Ts);
% 
% mdl = greyest(data0,mdlstruct,greyestOptions('InitialState','zero','DisturbanceModel','estimate'));
% 
% figure(8)
% compare(data0,mdl,1);
% 
% figure(9)
% resid(data0,mdl)

%% Validation Data 1: remaining data
figure(32)
subplot(2,1,1);
compare(data1,mdl,1);
subplot(2,1,2);
resid(data1,mdl)

%% Validation Data 2: new dataset
figure(33)
subplot(2,1,1);
compare(data2,mdl,1);
subplot(2,1,2);
resid(data2,mdl)

%% Plot Zeros and Poles
figure(34)
showConfidence(iopzplot(mdl),1)
axis([-1.1 1.1 -1.1 1.1])