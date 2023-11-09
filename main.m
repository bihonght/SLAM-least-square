clc 
clear all
close all

global size_state_vector
global num_landmarks
global num_poses
% pose of robots 

r0 = [0; 0; 0]
r1 = [6; 2; pi/6]
r2 = [8.5; 4.5; pi/4] 
r3 = [6; 9; pi]
r4 = [1; 10; -2*pi/3]

% landmarks

f1 = [-0.4; 5]
f2 = [5; 3.5]
f3 = [3; 11]

%% define global variables

global robot_poses; 
robot_poses = [r0'; r1'; r2'; r3'; r4'] 

global landmarks;
landmarks = [f1'; f2'; f3'];

size_state_vector = length(robot_poses)*3 + length(landmarks)*2;
num_landmarks = length(landmarks); 
num_poses = length(robot_poses);

%% Compute SLAM Gauss Newton method

X = zeros(num_poses,3);
Z = ones(num_landmarks,2);

% At first, I flat the X variable {in F(X)} to calculate but it doesnt work
% since Jacobian matrix has size of mxn with m<n.
%
% Then, I change the X variable size (not flatten) and change the
% Jacobian matrix formula. 
% J_H = [dh1/dx_i  dh1/dy_i    dh1/dtheta_i
%        dh2/dx_i  dh2/dy_i    dh2/dtheta_i
%        dh2/dx_i  dh2/dy_i    dh2/dtheta_i]


for t=1:5 
        [X,Z] = slam(X,Z)


% [X,Z] = slam(4,2,X,Z)

%% Plot
% Create a figure
figure;
hold on;

% Plot the landmarks
plot(landmarks(:, 1), landmarks(:, 2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

plot(Z(:, 1), Z(:, 2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

% Plot the robot poses
quiver(robot_poses(:, 1), robot_poses(:, 2), cos(robot_poses(:, 3)), sin(robot_poses(:, 3)), 0.2, 'b', 'filled', 'DisplayName', 'Robot Poses');

quiver(X(:, 1), X(:, 2), cos(X(:, 3)), sin(X(:, 3)), 0.2, 'r', 'filled', 'DisplayName', 'Robot Poses');

% Connect the robot poses with dashed lines
for i = 1:size(robot_poses, 1) - 1
    line([robot_poses(i, 1), robot_poses(i+1, 1)], [robot_poses(i, 2), robot_poses(i+1, 2)], 'Color', 'b', 'LineStyle', '--');
end

% Set axis limits and labels
xlim([-2, 15]); % Adjust the limits as needed
ylim([-2, 15]);
xlabel('X-axis');
ylabel('Y-axis');
            
% Add a legend
legend('Landmarks', 'Estimate Landmarks', 'Robot Poses', 'Estimated Poses', 'Location', 'Best');

% Title and grid (optional)
title(['Iteration ', num2str(t)]);
grid on;

% Hold off to stop adding to the current plot
hold off;

end





