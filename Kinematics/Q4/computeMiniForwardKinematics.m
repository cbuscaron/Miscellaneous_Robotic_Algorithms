function [endeff] = computeMiniForwardKinematics(rads1,rads2)

beta = (rads1-rads2);
alpha=(1/2)*(rads1 + rads2)+pi;

L2 = 2;
L1 = 1;
r = L2 - L1*cos(beta);

endeff = [r*cos(alpha), r*sin(alpha)];