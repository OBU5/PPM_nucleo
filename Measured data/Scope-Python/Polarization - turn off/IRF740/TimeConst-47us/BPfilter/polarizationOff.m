C1_x = table2array(C1Trace00002(:,1));
C1_y = table2array(C1Trace00002(:,2));

C2_x = table2array(C2Trace00002(:,1));
C2_y = table2array(C2Trace00002(:,2));

C3_x = table2array(C3Trace00002(:,1));
C3_y = table2array(C3Trace00002(:,2));
C4_x = table2array(C4Trace00002(:,1));
C4_y = table2array(C4Trace00002(:,2));


%Senzor - zivy
subplot(4,1,1)
plot(C1_x,C1_y);
%senzor - zem

subplot(4,1,2)
plot(C2_x,C2_y);
%S5

subplot(4,1,3)
plot(C3_x,C3_y);
%S4

subplot(4,1,4)
plot(C4_x,C4_y);


fig = plt.figure()
gs = gridspec.GridSpec(2, 1, height_ratios=[2, 1]) 
ax0 = plt.subplot(gs[0])
ax0.set_yscale("log")
line0, = ax0.plot(x, y, color='r')




