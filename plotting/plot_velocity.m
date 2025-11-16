clc; close all, clear;
filename = 'C:/Users/awolo/Documents/Uia,2025 Autumn/MAs246-Motion control/putty.txt';

fid = fopen(filename,'r');
if fid == -1
    error('Cannot open file: %s', filename);
end

pos = [];
vel = [];

tline = fgetl(fid);
while ischar(tline)
    % Matches numbers with optional minus sign and decimal point
    tokens = regexp(tline, 'Position:\s*(-?[\d\.]+), Velocity:\s*(-?[\d\.]+)', 'tokens');
    if ~isempty(tokens)
        nums = str2double(tokens{1});
        pos(end+1) = nums(1);
        vel(end+1) = nums(2);
    end
    tline = fgetl(fid);
end

fclose(fid);

dt = 0.01;  % sampling interval in seconds (replace with your actual rate)
t = (0:length(pos)-1) * dt;

% Plot position vs time
figure;
plot(t, pos, LineStyle="-",LineWidth=3);
xlabel('Time (s)',FontSize= 14,FontWeight='bold');
ylabel('Position','FontSize',14,'FontWeight','bold');
title('Position vs Time',FontSize=14,FontWeight='bold');
ylim([0 7])
grid on;
legend("Elevator movement",FontSize=10,FontWeight="bold")

% Plot velocity vs time
figure;
plot(t, vel, '-',LineWidth=3);
xlabel('Time (s)',FontSize= 14,FontWeight='bold');
ylabel('Velocity','FontSize',14,'FontWeight','bold');
title('Velocity vs Time',FontSize=14,FontWeight='bold');
ylim([-0.7 0.7])
grid on;
legend("Speed of elevator",FontSize=10,FontWeight="bold")


