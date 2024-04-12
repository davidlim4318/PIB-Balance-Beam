clear
tlim = [0 10];

G = tf(40.6502,[1 5.0413e-04 0]);

G_P = G;

figure(1)
subplot(2,5,1)
rlocus(G_P)
axis equal
shg
subplot(2,5,6)
bode(G_P)
shg

%% Plant with Delay
d = 0.15;
[num,den] = pade(d,2);
P_pade = tf(num,den);

G_P = G*P_pade;

figure(1)
subplot(2,5,1)
rlocus(G_P)
axis equal
shg
subplot(2,5,6)
bode(G_P)
shg

%% Lead Controller 1
omega_target = 0.8;
alpha = 100;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);
D_lead = tf([1 z],[1 p]);

D1 = D_lead;

figure(1)
subplot(2,5,2)
rlocus(G_P*D1)
axis equal
shg
subplot(2,5,7)
bode(G_P*D1)
shg

D2 = D1;

%% Lag Controller 1
omega_target = 0.01;
alpha = 0.1;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);
D_lag = tf([1 z],[1 p]);

D2 = D1*D_lag;

figure(1)
subplot(2,5,3)
rlocus(G_P*D2)
axis equal
shg
subplot(2,5,8)
bode(G_P*D2)
shg

%% Adjust Gain
omega_target = 1.7;

K = 1/abs(freqresp(G_P*D2,omega_target));
D3 = K*D2;

poles = rlocus(G_P*D2,K);
figure(1)
subplot(2,5,3)
rlocus(G_P*D2)
axis equal
hold on
plot(real(poles),imag(poles),'r*')
hold off

subplot(2,5,8)
bode(G_P*D3)
shg

%% Closed Loop
T = G_P*D3/(1+G_P*D3);

pole(T)

figure(1)
subplot(2,5,4)
rlocus(T,1)
axis equal

subplot(2,5,5)
step(T,tlim)
shg

subplot(2,5,9)
bode(T)
shg

S_u = D3/(1+G_P*D3);

subplot(2,5,10)
step(S_u,tlim)
shg

%% Test With Different Transfer Function
G1 = tf(60,[1 0 0]);
G_P1 = G1*P_pade;
T1 = G_P1*D3/(1+G_P1*D3);

pole(T1)

figure(2)
subplot(2,2,1)
rlocus(T1,1)
axis equal

subplot(2,2,2)
step(T1,tlim)
shg

subplot(2,2,3)
bode(T1)
shg

S_u1 = D3/(1+G_P1*D3);

subplot(2,2,4)
step(S_u1,tlim)
shg

%% Difference Equation 1 (WITHOUT K)
h = 0.025;
opts = c2dOptions('PrewarpFrequency',omega_target);
D_z = c2d(D2,h,opts)

syms z U(z) E(z) theta_k x_k
[num, den] = tfdata(D_z);
eqn = simplify( poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*U(z) );
eqn_diff = vpa(iztrans(eqn))
K

save('loop_shaping_1')
