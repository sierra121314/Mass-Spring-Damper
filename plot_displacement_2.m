clear all, clc, close all
% When importing new data

data1 = load('x_history.txt');
data2 = load('x_dot_history.txt');
data3 = load('x_dd_history.txt');
data4 = load('P_force_history.txt');
data5 = load('A_force_history.txt');
data6 = load('P_best_fitness_history.txt');
data7 = load('A_best_fitness_history.txt');

time = [1:500];
generations = [1:100];

subplot(7,1,1)
plot(time, data1)
title('displacement')
ylabel('displacement')
xlabel('time[ms]') 

subplot(7,1,2)
plot(time, data2)
title('velocity')
ylabel('velocity')
xlabel('time[ms]')

subplot(7,1,3)
plot(time, data3)
title('acceleration')
ylabel('acceleration')
xlabel('time[ms]')

subplot(7,1,4)
plot(time, data4)
title('Protagonist force')
ylabel('P force')
xlabel('time[ms]')

subplot(7,1,5)
plot(time, data5)
title('Antagonist force')
ylabel('A force')
xlabel('time[ms]')

subplot(7,1,6)
plot(generations, data6)
title('Best Protagonist fitness')
ylabel('best fitness so far')
xlabel('generations')

subplot(7,1,7)
plot(generations, data7)
title('Best Antagonist fitness')
ylabel('best fitness so far')
xlabel('generations')
