C1_x = table2array(C1Trace00000(3600000:end-1600000,1))+2.5-1.8;
C1_y = table2array(C1Trace00000(3600000:end-1600000,2));

C2_x = table2array(C2Trace00000(3600000:end-1600000,1))+2.5-1.8;
C2_y = table2array(C2Trace00000(3600000:end-1600000,2))-2.5;

C3_x = table2array(C3Trace00000(3600000:end-1600000,1)) + 2.5-1.8;
C3_y = table2array(C3Trace00000(3600000:end-1600000,2)) -2.5;

plot(C1_x,C1_y);
hold on
plot(C2_x,C2_y);
hold on
plot(C3_x,C3_y);