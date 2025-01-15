clear

%% Simulation Example
Ts = 0.025;

% zG = [];
% pG = [0 -0.01 -10 -10];
% kG = 9.8*1000/180*pi/1.7/2 * 10^2;
% Gc = zpk(zG,pG,kG);

zGc = [];
pGc = [0 -1 -10];
kGc = 9.8*1000/180*pi/1.7/2 * 10;
Gc = zpk(zGc,pGc,kGc);

G0 = c2d(Gc,Ts,'zoh');

%% Controller
% zF = 1;
% pF = 0.5;
% kF = 0.5;

zF = [1 0.9];
pF = [0.1 0.999];
kF = 1;
F = zpk(zF,pF,kF,Ts);

figure(42)
bode(F)

figure(1)
rlocus(F*G0)
axis([0.6 1.2 -0.2 0.2])
axis equal

%% Closed-Loop System
Gcl = minreal(G0*F/(1 + G0*F));
S = minreal(1/(1 + G0*F));

z = zero(Gcl);
p = pole(Gcl);

figure(1)
hold on
plot(pole(Gcl),'r*')
hold off

figure(2)
step(Gcl)

%% Input Signal
N = 1200;
T = Ts*N;

% rm = 20;
% seq = idinput(N,'prbs',[0 1/2],[0 1]);
% r = rm*(2*seq-1);

r = 10*(2*randi([0 1],N,1)-1);

% figure(3)
% plot(r)

R = fft(r);
Phir = abs(R).^2/N;
w = linspace(0,pi,N/2+1);

figure(4)
plot(w,Phir(1:N/2+1))

% xcorr(r,0,'biased')
% trapz(w,Phir(1:N/2+1))/pi

% allOneString = sprintf('%.0f,',seq)

%% Simulate Noise
v = sqrt(8)*randn(N,1);

vf = lsim(S,v);

% figure(5)
% plot([v vf])

% V = fft(v);
% Phiv = abs(V).^2/N;
% w = linspace(0,pi,N/2+1);
% 
% figure(4)
% hold on
% plot(w,Phiv(1:N/2+1))
% hold off

%% Simulate Output
H = zpk([],cell2mat(Gcl.P),1,Ts);

ynf = lsim(Gcl,r);
y = ynf + vf;

figure(6)
plot([ynf y])

%% FIR LS
n = 140;
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

%%
imp = impulse(Gcl,n*Ts);

figure(5)
clf
fill([k flip(k)],[ci(:,1);flip(ci(:,2))]/Ts,'b','FaceAlpha',0.2,'LineStyle','none')
hold on
plot(k,theta_FIR/Ts,'b-');
plot(k,imp)
hold off

%% Hankel Matrix
N1 = n/2;
H = hankel(theta_FIR(2:N1+1),theta_FIR(N1+1:2*N1-1));
[U,S,V] = svd(H);
sigma = diag(S);

m = 30;
figure(7)
tiledlayout(2,1)
nexttile(1)
plot(sigma(1:m),'r*');
nexttile(2)
semilogy(sigma(1:m),'r*');

%% Parameter Estimation Using PEM (Grey-Box Model)
data = iddata(y,r,Ts);
a = G0.P{1};
par = [G0.Z{1}; a(2:3); G0.K];
aux = [zF(2) pF kF];
mdlstruct = idgrey('dynamicsCLO4',par,'d',aux,Ts);

mdl = greyest(data,mdlstruct,greyestOptions('InitialState','zero'));

% mdl = greyest(data,mdlstruct,greyestOptions('InitialState','zero','DisturbanceModel','estimate'));
% (since model is ARMAX, poles still converge, but zeros change)

%% Model Validation
figure(9)
compare(data,mdl);  % simulation, no disturbances
hold on
plot(0:Ts:T-Ts,ynf)
hold off

%%
figure(10)
showConfidence(iopzplot(mdl),1)
hold on
plot(real(z),imag(z),'rs')
plot(real(m),imag(m),'r*')
hold off
axis([-1.1 1.1 -1.1 1.1])

figure(11)
resid(data,mdl)