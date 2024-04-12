clear
tlim = [0 10];
n_col = 6;

G = tf(40.6502,[1 5.0413e-04 0]);

G_P = G;

figure(1)
subplot(2,n_col,1)
rlocus(G_P)
axis equal
shg
subplot(2,n_col,n_col+1)
bode(G_P)
shg

%% Plant with Delay
d = 0.15;
[num,den] = pade(d,2);
P_pade = tf(num,den);

G_P = G*P_pade;

col = 1;
figure(1)
subplot(2,n_col,col)
rlocus(G_P)
axis equal
shg
subplot(2,n_col,n_col+col)
bode(G_P)
shg

%% Lead Controller
omega_target = 1.1;
alpha = 200;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);
D_lead = tf([1 z],[1 p]);

D1 = D_lead;

col = 2;
figure(1)
subplot(2,n_col,col)
rlocus(G_P*D1)
axis equal
shg
subplot(2,n_col,n_col+col)
bode(G_P*D1)
shg

figure(2)
clf
hold on
bode(D1)
shg

D2 = D1;

%% Lag Controller
omega_target = 0.001;
alpha = 1/200;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);
D_lag = tf([1 z],[1 p]);

D2 = D1*D_lag;

col = 3;
figure(1)
subplot(2,n_col,col)
rlocus(G_P*D2)
axis equal
shg
subplot(2,n_col,n_col+col)
bode(G_P*D2)
shg

figure(2)
clf
hold on
bode(D1)
bode(D2)
shg

D3 = D2;

%% Low-Pass Filter
omega_c = 50;

D_lpf = tf(omega_c,[1 omega_c]);

D3 = D2*D_lpf;

col = 4;
figure(1)
subplot(2,n_col,col)
rlocus(G_P*D3)
axis equal
shg
subplot(2,n_col,n_col+col)
bode(G_P*D3)
shg

figure(2)
clf
hold on
bode(D1)
bode(D2)
bode(D3)
shg

%% Adjust Gain
omega_target = 1.7;

K = 1/abs(freqresp(G_P*D3,omega_target));
D4 = K*D3;

poles = rlocus(G_P*D3,K);

col = 4;
figure(1)
subplot(2,n_col,col)
rlocus(G_P*D3)
axis equal
hold on
plot(real(poles),imag(poles),'r*')
hold off

subplot(2,n_col,n_col+col)
bode(G_P*D4)
shg

figure(2)
clf
hold on
bode(D1)
bode(D2)
bode(D3)
bode(D4)
shg

%% Closed Loop
T = G_P*D4/(1+G_P*D4);

%pole(T)

S_u = D4/(1+G_P*D4);

col = 5;
figure(1)
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

%% Test With Different Transfer Function
G1 = tf(60,[1 0 0]);
G_P1 = G1*P_pade;
T1 = G_P1*D4/(1+G_P1*D4);

%pole(T1)

S_u1 = D4/(1+G_P1*D4);

figure(3)
clf
subplot(2,2,1)
rlocus(T1,1)
axis equal

subplot(2,2,2)
step(T1,tlim)
shg

subplot(2,2,3)
bode(T1)
shg

subplot(2,2,4)
step(S_u1,tlim)
shg

%% Difference Equations for Controller, Low-Pass Filter and, K
h = 0.025;
opts = c2dOptions('PrewarpFrequency',omega_target);

D_2_z = c2d(D2,h,opts)

D_lpf_z = c2d(D_lpf,h)

syms z U(z) E(z) theta_k x_k
[num, den] = tfdata(D_2_z);
eqn = poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*U(z);
diff_eqn = vpa(iztrans(eqn))

[num, den] = tfdata(D_lpf_z);
eqn = poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*U(z);
diff_eqn_lpf = vpa(iztrans(eqn))

K

figure(4)
clf
hold on
bode(D_2_z)
bode(D_lpf_z*D_2_z)

%%
save('loop_shaping_1')
