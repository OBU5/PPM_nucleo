C1_x = table2array(C1Trace00002(:,1))+2.5-1.8;
C1_y = table2array(C1Trace00002(:,2));

C2_x = table2array(C2Trace00002(:,1))+2.5-1.8;
C2_y = table2array(C2Trace00002(:,2))-2.5;

C3_x = table2array(C3Trace00002(:,1)) + 2.5-1.8;
C3_y = table2array(C3Trace00002(:,2)) -2.5;
C4_x = table2array(C4Trace00002(:,1)) + 2.5-1.8;
C4_y = table2array(C4Trace00002(:,2)) -2.5;


%Senzor - zivy
plot(C1_x,C1_y);
hold on
%senzor - zem
plot(C2_x,C2_y);
hold on
%S5
plot(C3_x,C3_y);
hold on
%S4
plot(C4_x,C4_y);



