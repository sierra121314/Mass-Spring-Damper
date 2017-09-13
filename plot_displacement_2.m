close all
% When importing new data
% 1) Home tab
% 2) Clear Workspace - deletes variables
% 3) Import Data

data = load('x_history.txt')

subplot(3,1,1)
plot(vectorspot, x)
title('displacement')
ylabel('displacement')
xlabel('time[s]') %%policies?

subplot(3,1,2)
plot(vectorspot, x_dot)
title('velocity')
ylabel('velocity')
xlabel('time[s]')

subplot(3,1,3)
plot(vectorspot, x_dd)
title('acceleration')
ylabel('acceleration')
xlabel('time[s]')

