syms roll pitch r d


T1 =  [1 0          0         0;
       0 cosd(roll) -sind(roll) 0;
       0 sind(roll)  cosd(roll) 0;
       0 0          0         1];
 
T2 =  [cosd(pitch)  0 sind(pitch) 0;
       0           1 0          0;
       -sind(pitch) 0 cosd(pitch) 0;
       0           0 0          1];

T3 =  [1 0 0 0;
       0 1 0 0;
       0 0 1 r;
       0 0 0 1];

T4E1 =   [cosd(120) -sind(120) 0 0;
        sind(120)  cosd(120) 0 0;
        0          0         1 0;
        0          0         0 1];

T4E2 =   [cosd(240) -sind(240) 0 0;
        sind(240)  cosd(240) 0 0;
        0          0         1 0;
        0          0         0 1];

T5 =  [1 0 0 -d;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

E1 = T1*T2*T3*T4E1*T5;
E2 = T1*T2*T3*T4E2*T5;
E1 = E1(1:3,4);
E2 = E2(1:3,4);

T1*T2*T3