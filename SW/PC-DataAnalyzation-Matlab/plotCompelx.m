clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;
fontSize = 20;
t = linspace(0, 40*pi, 1000);
j = sqrt(-1);
complexSignal = exp(-t/45).*(cos(t) + j*sin (t))
% Get the real component and plot it on the x axis
x = real(complexSignal);
y = imag(complexSignal);
plot3(x, t, y, '-g','LineWidth',5);
grid on
hold on
plot3(x, t, zeros(1000)-2,'-r','LineWidth',5);
plot3(zeros(1000)-2, t, y,'-b','LineWidth',5);
xlim([-2,2])
zlim([-2,2])
view(135,30);
xlabel('Reálná ?ást', 'Rotation',20,'FontSize', 25) 
ylabel('?as', 'Rotation',-20,'FontSize', 25) 
zlabel('Imaginární ?ást','FontSize', 25) 



% format long g;
% format compact;
% fontSize = 20;
% start = 20001
% num = 21000
% plot3(real(sigiq(start:num)), (start:num), imag(sigiq(start:num)), '-g','LineWidth',5);
% grid on
% hold on
% plot3(real(sigiq(start:num)), (1:1000), zeros(1000)-2,'-r','LineWidth',5);
% plot3(zeros(1000)-2, (1:1000), imag(sigiq(start:num)),'-b','LineWidth',5);
% xlim([-2,2])
% zlim([-2,2])
% view(135,30);
% xlabel('Reálná ?ást', 'Rotation',20,'FontSize', 25) 
% ylabel('?as', 'Rotation',-20,'FontSize', 25) 
% zlabel('Imaginární ?ást','FontSize', 25) 

