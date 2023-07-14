% 从txt文本文件读取数据
data = dlmread('log_0.txt'); % 假设数据存储在名为data.txt的文本文件中

% 数据点数目
numPoints = size(data, 1);

% 创建时间间隔
time = (0:numPoints-1) * 0.01; % 时间间隔为0.001秒

% 提取参考位置和实际位置
refPos = data(:, 1:3); % 参考位置
actualPos = data(:, 4:6); % 实际位置

% 提取参考速度和实际速度
refVel = data(:, 7:9); % 参考速度
actualVel = data(:, 10:12); % 实际速度

% 提取参考角度和实际角度
refAngles = data(:, 13:15); % 参考角度（欧拉角）
actualAngles = data(:, 16:18); % 实际角度（欧拉角）

% 绘制位置变化对比图
figure;
subplot(3, 1, 1);
plot(time, refPos(:, 1), 'r-', 'LineWidth', 2); % 绘制参考位置的X分量
hold on;
plot(time, actualPos(:, 1), 'b-', 'LineWidth', 2); % 绘制实际位置的X分量
xlabel('时间');
ylabel('位置');
title('X分量位置变化');
legend('参考位置', '实际位置');
grid on;

subplot(3, 1, 2);
plot(time, refPos(:, 2), 'r-', 'LineWidth', 2); % 绘制参考位置的Y分量
hold on;
plot(time, actualPos(:, 2), 'b-', 'LineWidth', 2); % 绘制实际位置的Y分量
xlabel('时间');
ylabel('位置');
title('Y分量位置变化');
legend('参考位置', '实际位置');
grid on;

subplot(3, 1, 3);
plot(time, refPos(:, 3), 'r-', 'LineWidth', 2); % 绘制参考位置的Z分量
hold on;
plot(time, actualPos(:, 3), 'b-', 'LineWidth', 2); % 绘制实际位置的Z分量
xlabel('时间');
ylabel('位置');
title('Z分量位置变化');
legend('参考位置', '实际位置');
grid on;

% 调整子图之间的间距
subplotSpacing = 0.05; % 子图间距（百分比）
set(gcf, 'Units', 'Normalized');
subplotPositions = get(gcf, 'Position');
subplotPositions(4) = 1 - (3 * subplotSpacing);
subplotPositions(2) = subplotSpacing;
set(gcf, 'Position', subplotPositions);

% 绘制速度变化对比图
figure;
subplot(3, 1, 1);
plot(time, refVel(:, 1), 'r-', 'LineWidth', 2); % 绘制参考速度的X分量
hold on;
plot(time, actualVel(:, 1), 'b-', 'LineWidth', 2); % 绘制实际速度的X分量
xlabel('时间');
ylabel('速度');
title('X分量速度变化');
legend('参考速度', '实际速度');
grid on;

subplot(3, 1, 2);
plot(time, refVel(:, 2), 'r-', 'LineWidth', 2); % 绘制参考速度的Y分量
hold on;
plot(time, actualVel(:, 2), 'b-', 'LineWidth', 2); % 绘制实际速度的Y分量
xlabel('时间');
ylabel('速度');
title('Y分量速度变化');
legend('参考速度', '实际速度');
grid on;

subplot(3, 1, 3);
plot(time, refVel(:, 3), 'r-', 'LineWidth', 2); % 绘制参考速度的Z分量
hold on;
plot(time, actualVel(:, 3), 'b-', 'LineWidth', 2); % 绘制实际速度的Z分量
xlabel('时间');
ylabel('速度');
title('Z分量速度变化');
legend('参考速度', '实际速度');
grid on;

% 调整子图之间的间距
subplotSpacing = 0.05; % 子图间距（百分比）
set(gcf, 'Units', 'Normalized');
subplotPositions = get(gcf, 'Position');
subplotPositions(4) = 1 - (3 * subplotSpacing);
subplotPositions(2) = subplotSpacing;
set(gcf, 'Position', subplotPositions);

% 绘制参考角度和实际角度变化图
figure;
subplot(3, 1, 1);
plot(time, refAngles(:, 1), 'r-', 'LineWidth', 2); % 绘制参考角度的X分量
hold on;
plot(time, actualAngles(:, 1), 'b-', 'LineWidth', 2); % 绘制实际角度的X分量
xlabel('时间');
ylabel('角度');
title('roll');
legend('参考角度', '实际角度');
grid on;

subplot(3, 1, 2);
plot(time, refAngles(:, 2), 'r-', 'LineWidth', 2); % 绘制参考角度的Y分量
hold on;
plot(time, actualAngles(:, 2), 'b-', 'LineWidth', 2); % 绘制实际角度的Y分量
xlabel('时间');
ylabel('角度');
title('pitch');
legend('参考角度', '实际角度');
grid on;

subplot(3, 1, 3);
plot(time, refAngles(:, 3), 'r-', 'LineWidth', 2); % 绘制参考角度的Z分量
hold on;
plot(time, actualAngles(:, 3), 'b-', 'LineWidth', 2); % 绘制实际角度的Z分量
xlabel('时间');
ylabel('角度');
title('yaw');
legend('参考角度', '实际角度');
grid on;

% 调整子图之间的间距
subplotSpacing = 0.05; % 子图间距（百分比）
set(gcf, 'Units', 'Normalized');
subplotPositions = get(gcf, 'Position');
subplotPositions(4) = 1 - (3 * subplotSpacing);
subplotPositions(2) = subplotSpacing;
set(gcf, 'Position', subplotPositions);





