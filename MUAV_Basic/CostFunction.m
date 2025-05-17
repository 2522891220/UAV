function [F] = CostFunction(Pos1, UAV)

%% 路径转换
Pos = SphericalToCart(Pos1,UAV);

% Input solution
x = Pos.x;
y = Pos.y;
z = Pos.z;

% Start location
xs = UAV.Start(:,1);
ys = UAV.Start(:,2);
zs = UAV.Start(:,3);

% Final location
xf = UAV.Goal(:,1);
yf = UAV.Goal(:,2);
zf = UAV.Goal(:,3);

% 完整路径包含了起点和终点
x_all = [ x ];
y_all = [ y ];
z_all = [ z ];

% 更新动态目标位置
for i = 1:UAV.num
    if UAV.dynamic_targets(i)
        % 计算预计截获时间
        dist = norm([x_all(i,end) - xs(i), y_all(i,end) - ys(i), z_all(i,end) - zs(i)]);
        intercept_time = dist / UAV.limt.v(i,2); % 使用最大速度估算
        
        % 更新目标位置
        new_pos = UAV.Goal(i,:) + UAV.target_velocity * intercept_time * UAV.movement_direction;
        
        % 更新终点位置
        xf(i) = new_pos(1);
        yf(i) = new_pos(2);
        zf(i) = new_pos(3);
    end
end


%% 编码转换
% 转换优化算法里的个体位置
UAVnum = UAV.num;                        % 无人机个数
% dim = UAV.PointDim;                      % 仿真维度
v = Pos.v;                               % 协同无人机速度 n个无人机就有n个速度
% P = Pos(1, 1:end-UAVnum);                % 协同无人机航迹 xyz xyz xyz ...

H = UAV.H;
% Track1 = cell(1,1);
a.V = v(1, :)';    % 该个体的速度 转置成[UAVnum 1]
% P_a = P(1, :);     % 该个体的航迹点矩阵
a.P = cell(UAVnum, 1);  % a.P含有UAVnum个元胞 用来保存每个UAV的航迹点坐标值，行为xyz 列为航迹点
for i =1:UAVnum
    P_ai=[x_all(i,:); y_all(i,:); z_all(i,:)];
    a.P(i) =  {P_ai};
end
Track1 = {a};

% Tracks
% 包括V和P两个元胞'
% V：[UAV.num 1] 是速度矩阵  保存了每个无人机的固定速度
% P：{
% 对于UAV.num个UAV,保存了UAV.num个UAV的航迹坐标，其中以{3*flight_num1}为例，每一列是一个航迹点的xyz坐标
% 3是三维
%       {3*flight_num1}
%       {3*flight_num2}
%       ...
%    }
%

% 表达式系数调整（将各项指标无量纲化）
p1 = 1;    % 燃料项（已除以最大航程）
p21 = 1;   % 高度项
p22 = 1;   % 低度项
p31 = 1.2; % 雷达威胁程度
p32 = 1.1; % 其余威胁
p4 = 1.2;  % 时间同步项
p5 = 1;    % 碰撞项

%% 航迹检测
Track = Track1{1};
report = TrackDetect(Track, UAV);
% 检测内容包括：偏角/倾角、航程长度、航程时间、威胁物（雷达火炮）穿越情况、禁飞区穿越情况。并根据这些情况，生成航迹点是否有问题的报告

%% 航程长度成本
ZZ = sum(UAV.limt.L);
MaxL_mt = ZZ(2);  % n条路线的起点到终点的路径长度之和
f_o = p1 * report.L_mt / MaxL_mt;  % 航程长度/起点到终点的距离

%% 航程高度成本
% 如果不在安全高度范围 ，就有会额外成本
f_h = 0;
for i = 1 : UAV.num
    Hmax = UAV.limt.h(i, 2);
    Hmin = UAV.limt.h(i, 1);
    prenum = (Hmin+Hmax)/2;
    for k = 1 : UAV.PointNum(i)
        z = Track.P{i}(3, k);
        if z>Hmax
            fk = 1;
        elseif z < Hmin
            fk = 1;
        else
            fk = abs(z-(prenum))/prenum;
        end
        f_h = f_h + fk;
    end
end


%% 威胁成本
% 雷达和火炮等
Threat1 = report.Threat1;%雷达区碰撞次数
Threat2 = report.Threat2;%火炮区碰撞次数
Threat1_D = report.Threat1_D;
Threat2_D = report.Threat2_D;
O_r = UAV.Menace.radar(: ,1:end-1);            %雷达坐标
O_o = UAV.Menace.other(: ,1:end-1);            %火炮坐标
f_t = 0;
for i = 1 : UAV.num

    for k = 1 : size(Threat1_D{i},1)
%         P = Track.P{i}(:, k)' ;  % 此时 P对应了第i个UAV， 第k个航迹点

        % 判断这个航迹点是否在雷达m监测半径内
        if Threat1{i}(k)==1
            for m = 1 : size(O_r, 1)
                %                 Rt_r = UAV.Menace.radar(m ,end); % 雷达监测最大半径
                %                 dr = norm(P - O_r(m, :));
                %                 if dr<Rt_r
                d = Threat1_D{i}(k,m);
                if d~=0
                    fk = p31 / (d)^4;
                    f_t = f_t + fk;
                end
                %                 end
            end
        end
        % 判断这个航迹点是否在火炮m攻击半径内
        if Threat2{i}(k)==1
            for m = 1 : size(O_o, 1)
                Rt_a = UAV.Menace.radar(m ,end); % 火炮攻击最大半径
                d = Threat2_D{i}(k,m);
                if d~=0
                    %                 da = norm(P - O_o(m, :));
                    %                 if da<Rt_a
                    fk = Rt_a^4 / (Rt_a^4 + (d)^4 );
                    f_t = f_t + fk;
                end
                %                 end
            end
        end
    end
end

%% 时间协同约束
f_m = 0;
map_select=UAV.Choose(1);
for i = 1 : UAV.num
    Li = report.L(i);
    % 计算UAV i 的最长/最短时间
    tmax = Li / UAV.limt.v(i,1);
    tmin = Li / UAV.limt.v(i,2);
    ti = report.time(i);
    tc = UAV.tc(map_select); % 设定的协同时间
    if tc <= tmax && tc >= tmin
        fk = 0;
    else
        fk = p4 * abs(ti - tc);
    end
    f_m = f_m + fk;
end
% if f_m ~=0
%     f_m = 1;
% end


%% 空间协同约束
f_c = p5 * report.col_times;
% if f_c ~= 0
%     f_c = 1;
% end


%% 最大航偏角/最大俯仰角
Angle = report.AngleProb ;
f_a = 0;
for i = 1 : UAV.num
    ANG = Angle{i};
    if isempty(find(ANG==1))
        ;
    else
        f_a = f_a + 1;
    end
end

%% 最小航迹段
Traj = report.TrajProb ;
f_tr = 0;
for i = 1 : UAV.num
    TR = Traj{i};
    if isempty(find(TR==1))
        ;
    else
        f_tr = f_tr + 1;
    end
end

%% 禁飞区
NFZ = UAV.NFZ;
NFZ_num = size(NFZ,1);
f_no = 0;
for k=1: UAV.num
    x_all = [UAV.Start(k,1) Track.P{k}(1,  :) UAV.Goal(k,1)];
    y_all = [UAV.Start(k,2) Track.P{k}(2,  :) UAV.Goal(k,2)];
    N = size(x_all,2);
    for i = 1:NFZ_num
        NFZ_zone = NFZ(i,:);
        NFZ_x = NFZ_zone(1);
        NFZ_y = NFZ_zone(2);
        NFZ_radius = NFZ_zone(4);
        for j = 1:N-1
            % Distance between projected line segment and threat origin
            dist = DistP2S([NFZ_x NFZ_y],[x_all(j) y_all(j)],[x_all(j+1) y_all(j+1)]);
            if dist > (NFZ_radius) % No collision
                NFZ_cost = 0;
            elseif dist < (NFZ_radius)  % Collision
                NFZ_cost = 1;
            end
            f_no = f_no + NFZ_cost;
        end
    end
end

%% 目标函数分量
subF = [ f_o; f_h; f_t; f_m; f_c ;f_a;f_tr;f_no];

%% 目标权重
weight = [ 0.05, 0.05, 2, 0.7, 0.7 0.6 0.9 0.9]; % 默认权重


%% 加权目标函数
F = weight * subF ;

% % 输出信息
% Data.ProbPoint = report.ProbPoint;    % 所有有问题的点
% Data.AngleProb = report.AngleProb;    % 不满足角度约束的点
% Data.TrajProb = report.TrajProb;      % 不满足最小航迹间隔的点
% Data.Threat = report.Threat;          % 受威胁的点
%
% Data.L = report.L;                    % 每个无人机的航程
% Data.t = report.time;                 % 每个无人机的时间
% Data.c = report.col_times;            % 所有无人机总碰撞次数


end

