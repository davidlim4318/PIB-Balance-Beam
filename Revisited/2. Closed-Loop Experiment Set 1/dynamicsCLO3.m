function [A,B,C,D] = dynamicsCLO3(par,Ts,aux)  % no need for K, general SS is ARMAX
zG0 = par(1);
pG0 = par(2);
kG0 = par(3);
G0 = zpk(zG0,pG0,kG0,Ts);
zF = aux(1);
pF = [aux(2) aux(3)];
kF = aux(4);
F = zpk(zF,pF,kF,Ts);
Gcl = minreal(G0*F/(1 + G0*F));
[A,B,C,D] = zp2ss(Gcl.Z{1},Gcl.P{1},Gcl.K);
end