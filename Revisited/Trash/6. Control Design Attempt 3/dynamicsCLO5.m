function [A,B,C,D] = dynamicsCLO5(par,Ts,aux)  % no need for K, general SS is ARMAX
num = par(1);
den = [1 par(2) par(3) par(4)];
G0 = zpk(zpk([],1,1,Ts)*tf(num,den,Ts));
zF = aux(1);
pF = aux(2);
kF = aux(3);
F = tf(zpk(zF,pF,kF,Ts));
Gcl = minreal(G0*F/(1 + G0*F));
[A,B,C,D] = zp2ss(Gcl.Z{1},Gcl.P{1},Gcl.K);
end