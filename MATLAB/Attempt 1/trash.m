%% System TF
clear
tlim = [0 10];
n_col = 3;

G = tf(40.6502,[1 5.0413e-04 0]);

d = 0.15;
[num,den] = pade(d,2);
P_pade = tf(num,den);

G_P = G*P_pade;

%% Low-Pass Filter
omega_c = 50;

D_lpf = tf(omega_c,[1 omega_c]);

%% Notch Filter
omega_n = 2*pi/1.25;

D_not = tf([1 0 omega_n^2],conv([1 omega_n],[1 omega_n]));

%% PID
K_p = 0.05;
K_i = 0.005;
K_d = 0.04;

D_pid = tf([K_d K_p K_i],[1 0]);

col = 1;
figure(1)
subplot(2,n_col,col)
rlocus(G_P*D_pid)
axis equal
shg
subplot(2,n_col,n_col+col)
bode(G_P*D_pid)
shg

D = D_pid*D_lpf;

T = G_P*D/(1+G_P*D);

S_u = D/(1+G_P*D);

col = 2;
subplot(2,n_col,col)
rlocus(T,1)
axis equal

subplot(2,n_col,col+1)
step(T,tlim)
shg

subplot(2,n_col,n_col+col)
bode(T)
shg

subplot(2,n_col,n_col+col+1)
step(S_u,tlim)
shg

%% Lead-Lag
omega_target = 1;
alpha = 200;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);
D_lead = tf([1 z],[1 p]);

omega_target = 0.001;
alpha = 1/10000;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);
D_lag = tf([1 z],[1 p]);

D_ll = D_lead*D_lag;

omega_target = 2;

K = 1/abs(freqresp(G_P*D_ll,omega_target));
D_kll = K*D_ll;

col = 1;
figure(2)
subplot(2,n_col,col)
rlocus(G_P*D_kll)
axis equal
shg
subplot(2,n_col,n_col+col)
bode(G_P*D_kll)
shg

D = D_kll*D_lpf;

T = G_P*D/(1+G_P*D);

S_u = D/(1+G_P*D);

col = 2;
subplot(2,n_col,col)
rlocus(T,1)
axis equal

subplot(2,n_col,col+1)
step(T,tlim)
shg

subplot(2,n_col,n_col+col)
bode(T)
shg

subplot(2,n_col,n_col+col+1)
step(S_u,tlim)
shg


figure(10)
clf
hold on
bode(D_pid*D_lpf)
bode(D_kll*D_lpf)

%% Difference Equations
h = 0.025;
opts = c2dOptions('PrewarpFrequency',omega_target);

D_z = c2d(D_ll,h,opts)

D_lpf_z = c2d(D_lpf,h)

syms z U(z) E(z) theta_k x_k
[num, den] = tfdata(D_z);
eqn = poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*U(z);
diff_eqn = vpa(iztrans(eqn))

[num, den] = tfdata(D_lpf_z);
eqn = poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*U(z);
diff_eqn_lpf = vpa(iztrans(eqn))

K