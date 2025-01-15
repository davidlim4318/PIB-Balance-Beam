

%% Input Signal

N = 600;
Ts = 0.025;  % s
T = Ts*N;  % s

um = 15;
u = um*randn(N,1).*hann(N);

figure(1)
plot(u)

U = fft(u);
Phiu = abs(U).^2/N;
w = linspace(0,pi,N/2+1);

% Pu = periodogram(u,[],w);

figure(2)
plot(w,Phiu(1:N/2+1))
% hold on
% plot(w,Pu*2*pi,'--')
% hold off

xcorr(u,0,'biased')
trapz(w,Phiu(1:N/2+1))/pi

%% Simulation Example
num = [-0.0002 -0.0204 -1.0229 0.0503]*1e+04;
den = conv([1 0],[1.0000 271.4788 606.1776 102.9406]);
G0 = c2d(tf(num,den),Ts,'zoh');
M = bode(G0,w/Ts);

G0d = G0*tf([1 -1],1,Ts);
Md = bode(G0d,w/Ts);

%%
v = 5*randn(N,1);
ynf = lsim(G0,u);
y = ynf + v;

figure(3)
plot([ynf y])

%{

%% ETFE
Y = fft(y);
U = fft(u);
E = U(1:N/2+1).\Y(1:N/2+1);

[Eest, w] = tfestimate(u,y,rectwin(N),0,N);

figure(4)
loglog(w,abs(E));
hold on
loglog(w,abs(Eest),'x')
loglog(w,M(:))
hold off
xlabel("Frequency (rad/sample)")
ylabel("Magnitude")
title('Amplitude Bode plot')

%% SPA
Ruu = xcorr(u,u,N/2);
Ryu = xcorr(y,u,N/2);
gamma = N;
window = [zeros((N-gamma)/2,1); hann(gamma+1); zeros((N-gamma)/2,1)];
Ruu_weighted = Ruu.*window;
Ryu_weighted = Ryu.*window;
Suu = 1/N*fft([Ruu_weighted(N/2+1:N); Ruu_weighted(1:N/2)]);
Syu = 1/N*fft([Ryu_weighted(N/2+1:N); Ryu_weighted(1:N/2)]);
P = Suu(1:N/2+1).\Syu(1:N/2+1);

Pest = spa(u,y,gamma/2,w);

figure(5)
loglog(w,abs(P));
hold on
loglog(w,abs(Pest.ResponseData(:)),'x')
loglog(w,M(:))
hold off
xlabel("Frequency (rad/sample)")
ylabel("Magnitude")
title('Amplitude Bode plot')

%% ETFE with Prefiltering
yf = filter([1 -1],1,y);

Y = fft(yf);
U = fft(u);
E = U(1:N/2+1).\Y(1:N/2+1);

figure(4)
loglog(w,abs(E));
hold on
loglog(w,Md(:))
hold off
xlabel("Frequency (rad/sample)")
ylabel("Magnitude")
title('Amplitude Bode plot')

%% SPA with Prefiltering
yf = filter([1 -1],1,y);

Ruu = xcorr(u,u,N/2);
Ryu = xcorr(yf,u,N/2);
gamma = N;
window = [zeros((N-gamma)/2,1); hann(gamma+1); zeros((N-gamma)/2,1)];
Ruu_weighted = Ruu.*window;
Ryu_weighted = Ryu.*window;
Suu = 1/N*fft([Ruu_weighted(N/2+1:N); Ruu_weighted(1:N/2)]);
Syu = 1/N*fft([Ryu_weighted(N/2+1:N); Ryu_weighted(1:N/2)]);
P = Suu(1:N/2+1).\Syu(1:N/2+1);

figure(5)
loglog(w,abs(P));
hold on
loglog(w,Md(:))
hold off
xlabel("Frequency (rad/sample)")
ylabel("Magnitude")
title('Amplitude Bode plot')

%}