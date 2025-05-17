function [UAV] = Uav_Envir_Set(map_select, map_complexity, startPos, goalPos, flight_num, target_velocity, movement_direction)

% 作战环境与无人机部署设定
if map_select==1
    H = imread('ChrismasTerrain.tif');
    MAPSIZE_X = size(H,2); 
    MAPSIZE_Y = size(H,1); 
    MAPSIZE_Z = max(max(H));
    
    % ... [保持原有地图1的威胁设置不变]
    
elseif map_select==2
    load('TerrainData.mat');
    H = Final_Data/20;
    MAPSIZE_X = size(H,2);
    MAPSIZE_Y = size(H,1);
    MAPSIZE_Z = max(max(H));

    if map_complexity==1
        % 雷达和火炮
        R1=50;  % Radius
        x1 = 100; y1 = 150; z1 = 170; % center
        R2=60;  % Radius
        x2 = 250; y2 = 250; z2 = 190; % center
        R3=40;  % Radius
        x3 = 300; y3 = 100; z3 = 70; % center
        UAV.Menace.radar = [x1 y1 z1 R1;x2 y2 z2 R2];
        UAV.Menace.other = [x3 y3 z3 R3];

        % 禁飞区
        UAV.NFZ =  [180,370,300,60];
    else
        R1=40;  % Radius
        x1 = 100; y1 = 100; z1 = 120; % center
        R2=50;  % Radius
        x2 = 200; y2 = 200; z2 = 150; % center
        R3=50;  % Radius
        x3 = 300; y3 = 80; z3 = 170; % center
        R4=50;  % Radius
        x4 = 300; y4 = 340; z4 = 160; % center
        R5=40;  % Radius
        x5 = 200; y5 = 80; z5 = 150; % center
        R6=40;  % Radius
        x6 = 130; y6 = 280; z6 = 150; % center

        UAV.Menace.radar = [x1 y1 z1 R1; x4 y4 z4 R4; x5 y5 z5 R5;x6 y6 z6 R6];
        UAV.Menace.other = [x3 y3 z3 R3;x2 y2 z2 R2];

        % 禁飞区
        UAV.NFZ = [180,370,300,60];
    end
end

UAV.H = H;
UAV.Choose = [map_select,map_complexity];

% 无人机和目标设置
UAV.Start = startPos;
UAV.Goal = goalPos;
UAV.PointNum = flight_num;
UAV.PointDim = size(UAV.Start, 2);
UAV.num = size(UAV.Start, 1);

% 动态目标设置
UAV.target_velocity = target_velocity;
UAV.movement_direction = movement_direction;
UAV.dynamic_targets = [false; false; true; true]; % 标记哪些是动态目标

% 无人机约束设置（min,max)
UAV.limt.v = 0.34*repmat([0.3, 0.7], UAV.num, 1);           % 速度约束
UAV.limt.phi = deg2rad(repmat([-60, 60], UAV.num, 1));      % 偏角约束
UAV.limt.theta = deg2rad(repmat([-45, 45], UAV.num, 1));    % 倾角约束
UAV.limt.h = repmat([MAPSIZE_Z/2, MAPSIZE_Z*3/2], UAV.num, 1); % 高度约束
UAV.limt.x = repmat([1, MAPSIZE_X], UAV.num, 1);            % 位置x约束
UAV.limt.y = repmat([1, MAPSIZE_Y], UAV.num, 1);            % 位置y约束
UAV.limt.z = UAV.limt.h;                                     % 位置z约束

% 航程约束
UAV.limt.L = zeros(UAV.num, 2);
for i = 1:UAV.num
    if UAV.dynamic_targets(i)
        % 动态目标的航程约束需要考虑目标移动
        zz.max = 1.5 * norm(UAV.Goal(i, :) - UAV.Start(i, :));
    else
        % 静态目标的航程约束
        zz.max = norm(UAV.Goal(i, :) - UAV.Start(i, :));
    end
    zz.min = 2;
    UAV.limt.L(i, :) = [zz.min, zz.max];
end

% 多无人机协同设置
UAV.tc = [6000 4000 3000];  % 协同时间
UAV.ds = 25;                % 安全距离

end