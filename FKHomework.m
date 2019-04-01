L0 = 0.3;
L1 = 0.2;
L2 = 0.1;

t0 = 0.4;
t1 = 0.6;
t2 = 1.2;

t0d = rad2deg(t0);
t1d = rad2deg(t1);
t2d = rad2deg(t2);

xe = L0*cos(t0)+L1*cos(t0+t1)+L2*cos(t0+t1+t2)
ye = L0*sin(t0)+L1*sin(t0+t1)+L2*sin(t0+t1+t2)

R0 = [ cos(t0) -sin(t0) 0 0;
       sin(t0) cos(t0)  0 0;
       0       0        1 0;
       0       0        0 1];
R1 = [ cos(t1) -sin(t1) 0 0;
       sin(t1) cos(t1)  0 0;
       0       0        1 0;
       0       0        0 1];
R2 = [ cos(t2) -sin(t2) 0 0;
       sin(t2) cos(t2)  0 0;
       0       0        1 0;
       0       0        0 1];

D0 =[1 0 0 L0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
D1 =[1 0 0 L1;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
D2 =[1 0 0 L2;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

T0 = R0 * D0;
T1 = R1 * D1;
T2 = R2 * D2;

T = T0 * T1 * T2