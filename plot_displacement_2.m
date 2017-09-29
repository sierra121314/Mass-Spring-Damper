clear all, clc, close all
% When importing new data

data1 = load('x_history.txt');
data2 = load('x_dot_history.txt');
data3 = load('x_dd_history.txt');
data4 = load('P_force_history.txt');
data5 = load('A_force_history.txt');
data6 = load('P_best_fitness_history.txt');
data7 = load('A_best_fitness_history.txt');
data8 = load('ave_sensor_noise.txt');
data9 = load('tstep_sensor.txt');
data10 = load('tstep_actuator.txt');

time = [1:500];
generations = [1:100];

subplot(10,1,1)
plot(time, data1)
title('Displacement')
ylabel('Displacement')
xlabel('Time[ms]') 

subplot(10,1,2)
plot(time, data2)
title('Velocity')
ylabel('Velocity')
xlabel('Time[ms]')

subplot(10,1,3)
plot(time, data3)
title('Acceleration')
ylabel('Acceleration')
xlabel('Time[ms]')

subplot(10,1,4)
plot(time, data4)
title('Protagonist force')
ylabel('P force')
xlabel('Time[ms]')

subplot(10,1,5)
plot(time, data5)
title('Antagonist force')
ylabel('A force')
xlabel('time[ms]')

subplot(10,1,6)
plot(time, data9)
title('Sensor Noise for Last Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(10,1,7)
plot(time, data10)
title('Actuator Noise for Last Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(10,1,8)
plot(generations, data6)
title('Best Protagonist fitness')
ylabel('Best Fitness So Far')
xlabel('Generations')
axis([0 100 0 4000]);

subplot(10,1,9)
plot(generations, data7)
title('Best Antagonist fitness')
ylabel('Best Fitness So far')
xlabel('Generations')
axis([0 100 0 4000]);

subplot(10,1,10)
A = mean(data8,1)
plot(generations, A)
title('Average Sensor Noise per Generation')
ylabel('Noise')
xlabel('Generations')
