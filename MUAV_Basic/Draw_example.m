function Draw_example(UAV)
close all

map_select = UAV.Choose(1);
map_complexity = UAV.Choose(2);

%% 确定威胁物、火炮点、禁飞区、起点/终点 绘图区
figure

%% 地图与威胁物、火炮点、禁飞区
H = UAV.H;
MAPSIZE_X = size(H,2); 
MAPSIZE_Y = size(H,1); 

% 火炮和雷达
threats_radar = UAV.Menace.radar;
threats_other = UAV.Menace.other;

% 禁飞区
NFZ = UAV.NFZ;
NFZ_num = size(NFZ,1);

% 起点/终点（修改为4个无人机）
p1 = scatter3(UAV.Start(1,1),UAV.Start(1,2),UAV.Start(1,3),100,'bs','MarkerFaceColor','y'); hold on
p2 = scatter3(UAV.Goal(1,1),UAV.Goal(1,2),UAV.Goal(1,3),100,'kp','MarkerFaceColor','y');
p3 = scatter3(UAV.Start(2,1),UAV.Start(2,2),UAV.Start(2,3),100,'bs','MarkerFaceColor','y');
p4 = scatter3(UAV.Goal(2,1),UAV.Goal(2,2),UAV.Goal(2,3),100,'kp','MarkerFaceColor','y');
p5 = scatter3(UAV.Start(3,1),UAV.Start(3,2),UAV.Start(3,3),100,'bs','MarkerFaceColor','y');
p6 = scatter3(UAV.Goal(3,1),UAV.Goal(3,2),UAV.Goal(3,3),100,'kp','MarkerFaceColor','r'); % 动态目标用红色
p7 = scatter3(UAV.Start(4,1),UAV.Start(4,2),UAV.Start(4,3),100,'bs','MarkerFaceColor','y');
p8 = scatter3(UAV.Goal(4,1),UAV.Goal(4,2),UAV.Goal(4,3),100,'kp','MarkerFaceColor','r'); % 动态目标用红色

% 添加动态目标运动方向指示
quiver3(UAV.Goal(3,1),UAV.Goal(3,2),UAV.Goal(3,3), ...
    UAV.movement_direction(1)*30, UAV.movement_direction(2)*30, UAV.movement_direction(3)*30, ...
    'r','LineWidth',2);
quiver3(UAV.Goal(4,1),UAV.Goal(4,2),UAV.Goal(4,3), ...
    UAV.movement_direction(1)*30, UAV.movement_direction(2)*30, UAV.movement_direction(3)*30, ...
    'r','LineWidth',2);

legend([p1,p2,p3,p4,p5,p6,p7,p8], ...
    {'起点1','终点1(静态)','起点2','终点2(静态)', ...
     '起点3','终点3(动态)','起点4','终点4(动态)'});
hold on

% 绘制地形
[X,Y] = meshgrid(1:MAPSIZE_X,1:MAPSIZE_Y);
s = mesh(X,Y,H);
s.FaceColor = 'flat';
colormap summer;
set(gca, 'Position', [0 0 1 1]);
axis equal vis3d on;
shading interp;
material dull;
camlight left;
lighting gouraud;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
hold on

view(0,90)
box on
grid on

% [原有的威胁区域绘制代码保持不变]
end
