clc; clear; close all

Kp = 5;
Kd = 5;
Ki = 5;
max_clamp = 30;

% Solid object properties
m = 1;
g = 9.81;

% Set point
target = 400;
dt = 0.01;
n = 1000;

% Variable initialization
pos = 0;
vel = 0;
time = 0;
i = 0;
error_prev = 0;

% Arrays initialization
array_time(1:n + 1) = 0;
array_thrust(1:n + 1) = 0;
array_pos(1:n + 1) = 0;
array_target(1:n + 1) = 0;

for k = 1:n
    % PID computation
    error = target - pos;

    p = Kp * error;
    d = Kd * ((error - error_prev) / dt);
    i = i + Ki * error * dt;

    thrust = p;

    new_vel = vel + ((thrust / m) - g) * dt;
    new_pos = pos + vel * dt + (0.5 * ((thrust / m) - g)) * (dt * dt);

    % Update
    error_prev = target - pos;
    pos = new_pos;
    vel = new_vel;
    time = time + dt;

    array_time(k + 1) = time;
    array_thrust(k + 1) = thrust;
    array_pos(k + 1) = pos;
    array_target(k + 1) = target;
end

% Save results to txt file relative to this script
T = table(array_time, array_thrust, array_pos, array_target);
[filepath, ~, ~] = fileparts(mfilename('fullpath'));
writetable(T, fullfile(filepath, 'feedback_control.txt'));

% Display plot
figure()
plot(array_time, array_target, array_time, array_pos);
