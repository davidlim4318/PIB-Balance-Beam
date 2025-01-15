%% Load Data and Obtain Closed-Loop Transfer Functions
clear

load("mdlCLO3.mat")
% load("mdlCLO4.mat")
par = mdl.Report.Parameters.ParVector;

% A = mdl.A;
% B = mdl.B;
% C = mdl.C;
% D = mdl.D;
% K = mdl.K;
% [numGcl,denGcl] = ss2tf(A,B,C,D);
% Gcl = tf(numGcl,denGcl,Ts);
% [numHcl,denHcl] = ss2tf(A,K,C,1);  % accounting for feedthrough 1
% Hcl = tf(numHcl,denHcl,Ts);

[A,B,C,D,F] = polydata(mdl);  % coefficients of increasing powers of q^-1

Ts = 0.025;

Gcl = tf(B,conv(A,F),Ts);
Hcl = tf(C,conv(A,D),Ts);

%% Obtain Open-Loop Transfer Functions from Closed-Loop Transfer Functions
zG = [];
pG = [1 par(1)];
kG = par(2);
G = zpk(zG,pG,kG,Ts);  % coefficients of increasing powers of q^-1

% num = par(1);
% den = [1 par(2) par(3)];
% G = zpk(zpk([],1,1,Ts)*tf(num,den,Ts));  % coefficients of increasing powers of q^-1

% H = tf(C,1,Ts);  % WRONG

zF = 0.97;
pF = 0.5;
kF = 1;
F = zpk(zF,pF,kF,Ts);
S = minreal(1/(1+F*G));  % sensitivity function

H = zpk(zpk(Hcl).Z{1},S.Z{1},1,Ts);  % RIGHT
% tf(minreal(H*S))  %% compare with Hcl

[numG,denG] = tfdata(G);
[numH,denH] = tfdata(H);

mdlOL = idpoly(1,numG,numH,denH,denG,[],Ts);

%% Load Datasets
txt1 = readmatrix("expCL3.txt");
txt2 = readmatrix("expCL4.txt");
N = 1200;

data0CL = iddata(txt1(1:N,3),txt1(1:N,2),Ts);
data1CL = iddata(txt1(N+1:end,3),txt1(N+1:end,2),Ts);
data2CL = iddata(txt2(:,3),txt2(:,2),Ts);

data0OL = iddata(txt1(1:N,3),txt1(1:N,4),Ts);
data1OL = iddata(txt1(N+1:end,3),txt1(N+1:end,4),Ts);
data2OL = iddata(txt2(:,3),txt2(:,4),Ts);

%% Test OL Model with Input and Output Data (u,y) !!!
figure(1)
subplot(2,1,1);
compare(data0OL,mdlOL,1);
subplot(2,1,2);
resid(data0OL,mdlOL);

%%
figure(2)
subplot(2,1,1);
compare(data1OL,mdlOL,1);
subplot(2,1,2);
resid(data1OL,mdlOL);

%%
figure(3)
subplot(2,1,1);
compare(data2OL,mdlOL,1);
subplot(2,1,2);
resid(data2OL,mdlOL);

%% Controller Validation
e = data0CL.InputData - data0CL.OutputData;
u = lsim(F,e);
figure(4)
plot(data0OL.InputData)
hold on
plot(u,'.')
hold off