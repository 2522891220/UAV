function Opti_Curve_Plot(Iter_all,Curve_name)

MaxIt = size(Iter_all,2);
figure
colors = {
    [0, 0.75, 0.75],  % 青色
    [0, 0, 0],        % 黑色
    [1, 0, 0],        % 红色
    [0.75, 0, 0.75],  % 紫色
    [0, 0, 1],        % 蓝色
    [0, 0.5, 0],      % 绿色
    [0.93, 0.69, 0.13],% 橙色
    [0, 0, 0],        % 黑色
    [0.5, 0, 0.5],    % 粉色
    [0, 1, 0],        % 绿色
    [1, 0.5, 0],      % 橙色
    [0.75, 0.75, 0]   % 棕色
    };

pix = 5;
% 绘制30次运行的平均收敛曲线
plot([1:pix:MaxIt],Iter_all(1,1:pix:MaxIt), '-o', 'Color', colors{1}, 'LineWidth', 1,'MarkerSize',6);
hold on;
plot([1:pix:MaxIt],Iter_all(2,1:pix:MaxIt), '-+', 'Color', colors{2}, 'LineWidth', 1,'MarkerSize',6);
plot([1:pix:MaxIt],Iter_all(3,1:pix:MaxIt), '-*', 'Color', colors{12}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],Iter_all(1,:), '-.', 'Color', colors{4}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],Iter_all(1,:), '-p', 'Color', colors{5}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],Iter_all(1,:), '-o', 'Color', colors{6}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],Iter_all(1,:), '-s', 'Color', colors{7}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],mean(Convergence_curve_8(:,1:pix:MaxIt)), '--d', 'Color', colors{8}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],mean(Convergence_curve_9(:,1:pix:MaxIt)), '-^', 'Color', colors{9}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],mean(Convergence_curve_10(:,1:pix:MaxIt)), '--v', 'Color', colors{10}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],mean(Convergence_curve_11(:,1:pix:MaxIt)), '-->', 'Color', colors{11}, 'LineWidth', 1,'MarkerSize',6);
% plot([1:pix:MaxIt],mean(Convergence_curve_12(:,1:pix:MaxIt)), '-<', 'Color', colors{3}, 'LineWidth', 1,'MarkerSize',6);


% 添加图例
legend(Curve_name,'FontSize',12,'FontName','Times New Roman');

% 设置图标题和坐标轴标签
title('Multi-UAV Path Planning Fitness Curve','FontSize',12,'FontName','Times New Roman')
xlabel('Iteration','FontSize',12,'FontName','Times New Roman');
ylabel('Fitness Value','FontSize',12,'FontName','Times New Roman');
axis tight;
grid on;
box on;
hold off;

