%% Load Data and Obtain Closed-Loop Transfer Functions
clear

load("mdlCLO3.mat")
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
zG = par(1);
pG = [par(2) 1];
kG = par(3);
G = zpk(zG,pG,kG,Ts);  % coefficients of increasing powers of q^-1

% H = tf(C,1,Ts);  % WRONG

numF = [1 -1.9 0.9];
denF = [1 -1.099 0.0999];
F = tf(numF,denF,Ts);
S = minreal(1/(1+F*G));  % sensitivity function

H = zpk(zpk(Hcl).Z{1},S.Z{1},1,Ts);  % RIGHT
% tf(minreal(H*S))  %% compare with Hcl

[numG,denG] = tfdata(G);
[numH,denH] = tfdata(H);

mdlOL = idpoly(1,numG,numH,denH,denG,[],Ts);

%% Load Datasets
txt1 = readmatrix("expCL1.txt");
txt2 = readmatrix("expCL2.txt");
N = 1200;

data0CL = iddata(txt1(1:N,3)-135,txt1(1:N,2)-135,Ts);
data1CL = iddata(txt1(N+1:end,3)-135,txt1(N+1:end,2)-135,Ts);
data2CL = iddata(txt2(1:400,3)-130,txt2(1:400,2)-130,Ts);

data0OL = iddata(txt1(1:N,3)-135,txt1(1:N,4),Ts);
data1OL = iddata(txt1(N+1:end,3)-135,txt1(N+1:end,4),Ts);
data2OL = iddata(txt2(1:400,3)-130,txt2(1:400,4),Ts);

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
% u = e - 1.9*e1 + 0.9*e2 + 1.099*u1 - 0.0999*u2
numF = [1 -1.9 0.9];
denF = [1 -1.099 0.0999];

e = data0CL.InputData - data0CL.OutputData;
u = filter(numF,denF,e);
figure(3)
plot(data0OL.InputData)
hold on
plot(u,'.')
hold off