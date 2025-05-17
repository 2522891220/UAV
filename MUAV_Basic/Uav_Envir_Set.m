function [UAV] = Uav_Envir_Set(map_select,map_complexity,startPos,goalPos,flight_num)

% 作战环境与无人机部署设定

if map_select==1
    H = imread('ChrismasTerrain.tif');
    MAPSIZE_X = size(H,2); % x index: columns of H
    MAPSIZE_Y = size(H,1); % y index: rows of H
    MAPSIZE_Z = max(max(H));

    if map_complexity==1
        % 雷达和火炮
        R1=100;  % Radius
        x1 = 400; y1 = 500; z1 = 190; % center
        R2=100;  % Radius
        x2 = 600; y2 = 200; z2 = 220; % center
        R3=100;  % other
        x3 = 300; y3 = 250; z3 = 200; % center
        UAV.Menace.radar = [x1 y1 z1 R1;x2 y2 z2 R2];
        UAV.Menace.other = [x3 y3 z3 R3];

        % 禁飞区
        UAV.NFZ =  [700,500,200,100];
    else
        R1=90;  % Radius
        x1 = 400; y1 = 400; z1 = 210; % center
        R2=70;  % Radius
        x2 = 500; y2 = 170; z2 = 210; % center
        R3=60;  % Radius
        x3 = 200; y3 = 150; z3 = 220; % center
        R4=80;  % Radius
        x4 = 820; y4 = 300; z4 = 170; % center
        R5=90;  % Radius
        x5 = 350; y5 = 700; z5 = 190; % center
        R6=100;  % Radius
        x6 = 170; y6 = 350; z6 = 220; % center
        R7=90;  % Radius
        x7 = 620; y7 = 500; z7 = 240; % center

        UAV.Menace.radar = [x1 y1 z1 R1;x2 y2 z2 R2; x3 y3 z3 R3; x4 y4 z4 R4; x5 y5 z5 R5];
        UAV.Menace.other = [x6 y6 z6 R6;x7 y7 z7 R7];

        % 禁飞区
        UAV.NFZ = [650,270,200,80;800 600,150,90];
    end

elseif map_select==2
    load('TerrainData.mat');
    H = Final_Data/20;
    MAPSIZE_X = size(H,2); % x index: columns of H
    MAPSIZE_Y = size(H,1); % y index: rows of H
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
% 1 无人机航迹点设置
% （每行为一个无人机的参数）
UAV.Start = startPos;%[  0,           0, 0;
%0,           100,0;
%300,          0,0;      ];     % 起点位置(x,y,z)
UAV.Goal = goalPos;%[  875,       875,800;
%800,       875,800;
%875,       800,800;  ];         % 目标位置(x,y,z)
UAV.PointNum = flight_num;%[  26;
%24;
%24;  ];                 % 每个无人机导航点个数
UAV.PointDim = size(UAV.Start, 2);            % 坐标点维度
UAV.num = size(UAV.Start, 1);                 % UAV数量

% 2 无人机约束设置（min,max)
UAV.limt.v = 0.34*repmat([0.3, 0.7], UAV.num, 1);           % 速度约束 （0.3Ma ~ 0.7Ma）
UAV.limt.phi = deg2rad(repmat([-60, 60], UAV.num, 1));      % 偏角约束 （-60° ~ 60°）
UAV.limt.theta = deg2rad(repmat([-45, 45], UAV.num, 1));    % 倾角约束 （-45° ~ 45°）
UAV.limt.h = repmat([MAPSIZE_Z/2, MAPSIZE_Z*3/2], UAV.num, 1);                % 高度约束 （0.02km ~ 20km）
UAV.limt.x = repmat([1, MAPSIZE_X], UAV.num, 1);                  % 位置x约束 （0 ~ 875km）
UAV.limt.y = repmat([1, MAPSIZE_Y], UAV.num, 1);                  % 位置y约束 （0 ~ 875km）
UAV.limt.z = UAV.limt.h;                                    % 位置z约束 （忽略地球弧度）
UAV.limt.L = zeros(UAV.num, 2);                             % 航程约束
for i =1:UAV.num
    zz.max =  norm(UAV.Goal(i, :) - UAV.Start(i, :));
    zz.min = 2;
    UAV.limt.L(i, :) = [zz.min, zz.max];
end



% 5 多无人机协同设置
UAV.tc = [6000 4000 3000];        % 协同时间 （单位s）
UAV.ds = 25;          % 安全距离 （单位km）

end


