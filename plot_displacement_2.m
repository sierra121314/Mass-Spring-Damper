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
A = mean(data8,1);
plot(generations, A)
title('Average Sensor Noise per Generation')
ylabel('Noise')
xlabel('Generations')


%% TRAIN AND TEST
clear all, clc, close all
% When importing new data

data1 = load('x_history.txt'); 
data1t = load('test_x_history.txt');
data2 = load('x_dot_history.txt'); 
data2t = load('test_x_dot_history.txt');
data3 = load('x_dd_history.txt'); 
data3t = load('test_x_dd_history.txt');
data4 = load('P_force_history.txt'); 
data4t = load('test_P_force_history.txt');
data6 = load('P_best_fitness_history.txt'); 
data6t = load('test_P_best_fitness_history.txt');
data9 = load('tstep_sensor.txt');
data10 = load('tstep_actuator.txt');

time = [1:500];
generations = [1:100];
num_pol=[1:100];

subplot(7,2,1)
plot(time, data1)
title('Displacement')
ylabel('Displacement')
xlabel('Time[ms]') 

subplot(7,2,2)
plot(time, data1t)
title('Test Displacement')
ylabel('Test Displacement')
xlabel('Time[ms]') 

subplot(7,2,3)
plot(time, data2)
title('Velocity')
ylabel('Velocity')
xlabel('Time[ms]')

subplot(7,2,4)
plot(time, data2t)
title('Test Velocity')
ylabel('Test Velocity')
xlabel('Time[ms]')

subplot(7,2,5)
plot(time, data3)
title('Acceleration')
ylabel('Acceleration')
xlabel('Time[ms]')

subplot(7,2,6)
plot(time, data3t)
title('Test Acceleration')
ylabel('TestAcceleration')
xlabel('Time[ms]')
%%%%%%%%%%%%%%%%%

subplot(7,2,7)
plot(time, data4)
title('Protagonist force')
ylabel('P force')
xlabel('Time[ms]')

subplot(7,2,8)
plot(time, data4t)
title('Test Protagonist force')
ylabel('Test P force')
xlabel('Time[ms]')


subplot(7,2,9)
plot(generations, data6)
title('Best Protagonist fitness')
ylabel('Best Fitness So Far')
xlabel('Generations')

subplot(7,2,10)
plot(num_pol, data6t)
title('Test Protagonist Fitness(In Order)')
ylabel('Fitness of 100 Policies')
xlabel('Policy')

subplot(7,2,11)
plot(time, data9)
title('Sensor Noise for Last Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(7,2,12)
plot(time, data10)
title('Actuator Noise for Last Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

six=mean(data6);
sixt=mean(data6t);
