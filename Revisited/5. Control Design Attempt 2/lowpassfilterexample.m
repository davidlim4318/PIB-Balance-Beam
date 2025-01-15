Ts = 0.025;
T = 0.2;
L = tf(Ts/T,[1 Ts/T-1],Ts);

[num,den] = butter(1,Ts/T/pi);
L2 = tf(num,den,Ts);

figure(3)
bode(L)
hold on
bode(L2)
hold off