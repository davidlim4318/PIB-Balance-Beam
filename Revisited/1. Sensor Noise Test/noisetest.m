
clear
M = 50;

alpha = 0.01;
Nalpha = norminv(1-alpha/2,0,1);

N = 600;

%%
txt1 = readmatrix("noise1.txt");
t1 = txt1(:,1)/1000;
v1 = txt1(:,2);

N1 = length(v1);

mu1 = mean(v1)

v1 = v1 - mu1;

Rv1 = xcorr(v1,M,'biased');

figure(1)
plot(t1,v1)

CI1 = sqrt(sum(xcorr(v1,'biased').^2)/N)*Nalpha;

figure(2)
fill([-M M M -M],CI1*[1 1 -1 -1],'--','FaceAlpha',0.1);
hold on
stem(-M:M,Rv1)
hold off

V1 = fft(v1);
Phiv1 = abs(V1).^2/N1;
w = linspace(0,pi,N1/2+1);

figure(3)
plot(w,Phiv1(1:N1/2+1))

%%
txt2 = readmatrix("noise2.txt");
t2 = txt2(:,1)/1000;
v2 = txt2(:,2);

N2 = length(v2);

mu2 = mean(v2)

v2 = v2 - mu2;

Rv2 = xcorr(v2,M,'biased');

figure(1)
plot(t2,v2)

CI2 = sqrt(sum(xcorr(v2,'biased').^2)/N)*Nalpha;

figure(2)
fill([-M M M -M],CI2*[1 1 -1 -1],'--','FaceAlpha',0.1);
hold on
stem(-M:M,Rv2)
hold off

V2 = fft(v2);
Phiv2 = abs(V2).^2/N2;
w = linspace(0,pi,N2/2+1);

figure(3)
plot(w,Phiv2(1:N2/2+1))

%%
txt3 = readmatrix("noise3.txt");
t3 = txt3(:,1)/1000;
v3 = txt3(:,2);

N3 = length(v3);

mu3 = mean(v3)

v3 = v3 - mu3;

Rv3 = xcorr(v3,M,'biased');

figure(1)
plot(t3,v3)

CI3 = sqrt(sum(xcorr(v3,'biased').^2)/N)*Nalpha;

figure(2)
fill([-M M M -M],CI3*[1 1 -1 -1],'--','FaceAlpha',0.1);
hold on
stem(-M:M,Rv3)
hold off

V3 = fft(v3);
Phiv3 = abs(V3).^2/N3;
w = linspace(0,pi,N3/2+1);

figure(3)
plot(w,Phiv3(1:N3/2+1))

%%
alpha = 0.05;
Xalpha = chi2inv(1-alpha,M)

N1/Rv1(M+1)^2*sum(Rv1(M+2:end).^2)
N2/Rv2(M+1)^2*sum(Rv2(M+2:end).^2)
N3/Rv3(M+1)^2*sum(Rv3(M+2:end).^2)