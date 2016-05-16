function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;


rads2 = atan2(sqrt(1-((X^2+Y^2-2)/2)), ((X^2+Y^2-2)/2));

k2 = sin(rads2);
k1 = 1 + cos(rads2);
rads1 = atan2(Y, X) - atan2(k2, k1);