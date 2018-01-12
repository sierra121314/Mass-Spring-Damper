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

data6 = load('P_best_fitpergen_SR_history.txt'); 
data6t = load('test_P_best_fitness_history.txt');
data11 = load('A_force_history.txt');
data11t = load('test_A_force_history.txt');
data12 = load('A_best_fitpergen_SR_history.txt');

data9 = load('tstep_sensor.txt');
data10 = load('tstep_actuator.txt');
data9x = load('tstep_position.txt');
data9v = load('tstep_velocity.txt');
data9ave = load('ave_sensor_noise.txt');
data10ave = load('ave_actuator_noise.txt');
data9x_ave = load('ave_position_noise.txt');
data9v_ave = load('ave_velocity_noise.txt');

% data13 = load('stat_P_fitness.txt');
% data14 = load('stat_Ptest_fitness.txt');

time = [1:1000];
generations = [1:300];
num_pol=[1:20];
stat_runs = [1:5];

figure(1)
subplot(9,2,1)
plot(time, data1)
title('Displacement')
ylabel('Displacement')
xlabel('Time[ms]') 

subplot(9,2,2)
plot(time, data1t)
title('Test Displacement')
ylabel('Test Displacement')
xlabel('Time[ms]') 

subplot(9,2,3)
plot(time, data2)
title('Velocity')
ylabel('Velocity')
xlabel('Time[ms]')

subplot(9,2,4)
plot(time, data2t)
title('Test Velocity')
ylabel('Test Velocity')
xlabel('Time[ms]')

subplot(9,2,5)
plot(time, data3)
title('Acceleration')
ylabel('Acceleration')
xlabel('Time[ms]')

subplot(9,2,6)
plot(time, data3t)
title('Test Acceleration')
ylabel('TestAcceleration')
xlabel('Time[ms]')
%%%%%%%%%%%%%%%%%

subplot(9,2,7)
plot(time, data4)
title('Primary force')
ylabel('P force')
xlabel('Time[ms]')

subplot(9,2,8)
plot(time, data4t)
title('Test Primary force')
ylabel('Test P force')
xlabel('Time[ms]')

subplot(9,2,9)
plot(time, data11)
title('Antagonist force')
ylabel('A force')
xlabel('Time[ms]')

subplot(9,2,10)
plot(time, data11t)
title('Test Antagonist force')
ylabel('Test P force')
xlabel('Time[ms]')

subplot(9,2,11)
plot(generations, data6)
title('Best Protagonist fitness')
ylabel('Best Fitness So Far')
xlabel('Generations')

subplot(9,2,12)
plot(num_pol, data6t)
title('Test Protagonist Fitness(Sorted)')
ylabel('Fitness of 100 Policies')
xlabel('Policy')

subplot(9,2,13)
plot(generations, data12)
title('Best Antagonist fitness')
ylabel('Best Fitness So Far')
xlabel('Generations')

subplot(9,2,15)
plot(time, data9)
title('Sensor Noise per timestep for Last Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(9,2,16)
plot(time, data10)
title('Actuator Noise per timestep for Last Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(9,2,17)
plot(time, data9x)
title('Position Noise per timestep for Last Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(9,2,18)
plot(time, data9v)
title('Velocity Noise per timestep for Last Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')



%%%%%%%%%%%%FIGURE 2 - ZOOM IN OF NOISE %%%%%%%%%%%%%
figure(2)
subplot(3,2,1)
plot(time, data9)
title('Sensor Noise per timestep for Each Stat Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(3,2,2)
plot(time, data10)
title('Actuator Noise per timestep for Each Stat Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(4,2,3)
plot(time, data9x)
title('Position Noise per timestep for Each Stat Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(4,2,4)
plot(time, data9v)
title('Velocity Noise per timestep for Each Stat Run')
ylabel('Noise Magnitude')
xlabel('Time[ms]')

subplot(4,2,5)
plot(num_pol, data9ave)
title('Ave Sensor Noise for Each Stat Run')
ylabel('Noise Magnitude')
xlabel('Policy')

subplot(4,2,6)
plot(num_pol, data10ave)
title('Ave Actuator Noise for Each Stat Run')
ylabel('Noise Magnitude')
xlabel('Policy')

subplot(4,2,7)
plot(num_pol, data9x_ave)
title('Ave Position Noise for Each Stat Run')
ylabel('Noise Magnitude')
xlabel('Policy')

subplot(4,2,8)
plot(num_pol, data9v_ave)
title('Ave Velocity Noise for Each Stat Run')
ylabel('Noise Magnitude')
xlabel('Policy')

Pfit_mean=mean(data6);
Ptestfit_mean=mean(data6t);
Pfit_best=min(data6);
Ptestfit_best=min(data6t);


%%%% best vs median
figure(3)
data_med = load('P_med_fitpergen_SR_history.txt');
data_best = load('P_best_fitpergen_SR_history.txt');
subplot(2,2,[1,2])
plot(generations, data_med,'b')
hold on
plot(generations,data_best,'r')
title('Best and Median Primary Fitness')
ylabel('Fitness of Policies')
xlabel('Generations')
legend('Median Primary Fitness','Best Primary Fitness')

datat_med = load('test_P_med_fitpergen_SR_history.txt');
datat_best = load('test_P_best_fitpergen_SR_history.txt');
subplot(2,2,[3,4])
plot(stat_runs, datat_med)
hold on
plot(stat_runs,datat_best)
title('Best and Median Test Primary Fitness')
ylabel('Fitness of Policies')
xlabel('Stat Run')
legend('Median Test Primary Fitness','Best Test Primary Fitness')

% subplot(8,2,[21,22])
% plot(stat_runs,data13);
% hold on 
% plot(stat_runs,data14);
% Stat_P_mean = mean(data13) %average
% Stat_P_best = min(data13) %best fitness
% Stat_Ptest_mean = mean(data14) %average
% Stat_Ptest_best = min(data14) %best fitness
% plot(stat_runs,Stat_Ptest_mean*ones(size(stat_runs)));
% plot(stat_runs,Stat_Ptest_best*ones(size(stat_runs)));
% ylabel('Ave Fitness per Stat Run');
% xlabel('Stat runs');
% legend('Train Pro Fitness','Test Pro Fitness','Ave Test Pro Fitness','Best Test Pro Fit')
