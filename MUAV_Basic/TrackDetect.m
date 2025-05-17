function [report] = TrackDetect(Track, UAV)


%% 无人机航迹检测
% 威胁检测
dim = UAV.PointDim;                       % 仿真环境维度
M1 = [UAV.Menace.radar]; % 威胁区
M2 = [ UAV.Menace.other];

Threat1 = cell(UAV.num, 1);                % 威胁结果树
Threat2 = cell(UAV.num, 1);                % 威胁结果树
Threat1_D = cell(UAV.num, 1);   
Threat2_D = cell(UAV.num, 1);   
Angle = cell(UAV.num, 1);                 % 转角检测结果树
MiniTraj = cell(UAV.num, 1);              % 最小航迹片段检测结果树
ProbPoint = cell(UAV.num, 1);             % 问题点  

L = cell(UAV.num, 1);                     % 航迹片段树（累加结构）
Time = cell(UAV.num, 1);                  % 到达各个点时间树                                          
      
L_mt = 0;                                 % 所有无人机航迹之和
totalTime = zeros(UAV.num, 1);            % 所用时间
totalL = zeros(UAV.num, 1);               % 每个无人机飞行距离

for i = 1 : UAV.num
      PointNum = UAV.PointNum(i);   % 航迹点个数
      % 加上出发点和终点，共PointNum+1个片段
      Judge1 = zeros(1, PointNum+1);       % 记录与威胁区碰撞情况
      Judge2 = zeros(1, PointNum+1);       % 记录与威胁区碰撞情况
      L_i = zeros(1, PointNum+1);
      Time_i = zeros(1, PointNum+1);
      Angle_i = zeros(1, PointNum+1);     % 记录偏角和倾角约束违反情况
      Traj_i = zeros(1, PointNum+1);      % 记录航程长度约束违反情况
      ProbPoint_i = zeros(1, PointNum+1); % 记录航迹点是否存在问题
      % 进行检测
      l = 0;
%       Track
      V = Track.V(i); % 无人机i的速度
      for k = 1 : PointNum   % 对每个航迹点检测
            % 前向检测
            P2 = Track.P{i}(:,  k)' ;   % 转置成 1*dim  获取该航迹点的xyz坐标       
            if k == 1   % 如果是第一个航迹点 则其上一个点是起点
                P1 = UAV.Start(i, :);
                ZZ = P2 - P1;
                % 偏角检测
                phi0 = atan2(ZZ(2), ZZ(1));  
                phi1 = phi0;
                d_phi = phi1 - phi0; % 第一条航线由起点和第一个航迹点构成，此偏角为0
                % 倾角检测
                theta0 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                theta1 = theta0;
                d_theta = theta1 - theta0;  % 同上

            else  % 不是第一个航迹点 
                P1 = Track.P{i}(:, k-1)' ;  % 上一个点
                ZZ = P2 - P1;
                % 偏角检测
                phi1 = atan2(ZZ(2), ZZ(1));
                d_phi = phi1 - phi0;  % 由上一个航迹点与当前航迹点构成的航线 与 上一个航线对比
                phi0 = phi1;
                % 倾角检测
                theta1 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                d_theta = theta1 - theta0;
                theta0 = theta1;

            end

            % 威胁物检测
            [across, ~,RR] = CheckThreat(P1, P2, M1);
            Judge1(k) = across; % 该航线与威胁区碰撞次数
            R_Men1(k,:) = RR;
            [across, ~,RR] = CheckThreat(P1, P2, M2);
            Judge2(k) = across; % 该航线与威胁区碰撞次数
            R_Men2(k,:) = RR;

            % 计算前后航迹点之间的直线距离，以计算总长度和总用时
            dl = norm(P1 - P2);
            l = l + dl; % 累加距离
            t = l / V;  % 时间点
            L_i(k) = l; % 记录航线总距离
            Time_i(k) = t; % 记录用时

            % 倾角和偏角约束
            if abs(d_phi) > UAV.limt.phi(i, 2)  ||  abs(d_theta) > UAV.limt.theta(i, 2)
                Angle_i(k) = true; % 
            else
                Angle_i(k) = false;
            end

            % 总长度约束
            if dl < UAV.limt.L(i, 1) % 判断是否小于最短航程
                Traj_i(k) = true;
            else
                Traj_i(k) = false;
            end

            % 若有1（ture）处，则 该航迹点存在问题
            ProbPoint_i(k) = Angle_i(k) | Traj_i(k) | Judge1(k) | Judge2(k); 

            % 最后一段检测
            if k == PointNum
                P1 = UAV.Goal(i, :);

                % 威胁区情况 
                %                 [across, ~] = CheckThreat(P2, P1, M);
                %                 Judge(k+1) = across;
                [across, ~,RR] = CheckThreat(P1, P2, M1);
                Judge1(k+1) = across; % 该航线与威胁区碰撞次数
                R_Men1(k+1,:) = RR;
                [across, ~,RR] = CheckThreat(P1, P2, M2);
                Judge2(k+1) = across; % 该航线与威胁区碰撞次数
                R_Men2(k+1,:) = RR;
                % 航程长度与时间
                dl = norm(P1 - P2);
                l = l + dl;
                t = l / V;
                L_i(k+1) = l;
                Time_i(k+1) = t;

                % 倾角和偏角
                ZZ = P1-P2;
                phi1 = atan2(ZZ(2), ZZ(1));
                d_phi = phi1 - phi0;
                if dim>2      
                    theta1 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                    d_theta = theta1 - theta0;
                else
                    d_theta = 0;
                end

                if abs(d_phi) > UAV.limt.phi(i, 2)  ||  abs(d_theta) > UAV.limt.theta(i, 2)
                    Angle_i(k+1) = true;
                else
                    Angle_i(k+1) = false;
                end

                if dl < UAV.limt.L(i, 1)
                    Traj_i(k+1) = true;
                else
                    Traj_i(k+1) = false;
                end

                ProbPoint_i(k+1) = Angle_i(k+1) | Traj_i(k+1) | Judge1(k+1)| Judge2(k+1);

            end
      end   
      Threat1(i) = {Judge1};          % 威胁物碰撞 检测结果 
      Threat1_D(i) =  {R_Men1};
      Threat2(i) = {Judge2};          % 威胁物碰撞 检测结果
      Threat2_D(i) =  {R_Men2};
      Angle(i) = {Angle_i};         % 转角约束情况
      MiniTraj(i) = {Traj_i};       % 航迹片段 与最小航程对比结果
      ProbPoint(i) = {ProbPoint_i}; % 问题点 1为存在问题的航迹点 0为无问题

      L(i) = {L_i};                 % 路径长度（累加）
      Time(i) = {Time_i};           % 时间（累加）
      L_mt = L_mt + l;              % 总长度
      totalTime(i) = t;             % 时间
      totalL(i) = l;                % 长度
end

% 多无人机碰撞检测  碰撞  是在同一时间，UAV之间的距离小于安全距离
d_safe = UAV.ds; % 安全距离
CollideTimes = 0; % 碰撞次数
for i = 2 : UAV.num % 从第二个UAV开始判断，与前面的UAV进行碰撞检测，这样能够遍历所有UAV之间的碰撞情况
    PointNum_i = UAV.PointNum(i);

    % 对于无人机i的一个航迹，与前面的UAV的所有航迹进行碰撞检测，并依次遍历UAV i的航迹以实现检测
    for k = 1 : PointNum_i
        P1 = Track.P{i}(:, k)';  % K时刻 i 无人机位置
        t_i = Time{i, 1}(k);     % K时刻 i 无人机时间
        for j = 1 : i-1 % 与前面的UAV进行碰撞检测 
            PointNum_j = UAV.PointNum(j);
            flag = false; % 
            % 搜索同一时刻的点
            for kj = 1 : PointNum_j
                if kj ==1 % 
                    t_j_l = 0;
                    P_l =  UAV.Start(j, :); 
                else
                    t_j_l = Time{j}(kj-1);
                    P_l =  Track.P{j}(:, kj-1)'; 
                end
                t_j_r = Time{j}(kj);
                P_r =  Track.P{j}(:, kj)'; 

                if t_i <= t_j_r  &&  t_i >= t_j_l  % 判断无人机i的k航迹的时间是否在无人机j的航迹kj的区间内  如果在，则有碰撞的可能
                    flag = true;
                    P2 =  P_l + (t_i - t_j_l) / (t_j_r - t_j_l) * (P_r - P_l);% K时刻 J 无人机位置
                    % K时刻 j 无人机位置
                end
            end
            % ———————

            % 如果该片段有碰撞的可能，则检查两个无人机是否碰撞
            if flag  % 查找到P2位置，进行碰撞检测
                collide = CheckCollide(P1, P2, d_safe);
            else     % 没有P2位置，不会碰
                collide = false;
            end
            if collide
                CollideTimes = CollideTimes + 1;
            end
        end
    end
end

% 生成检测报告
report.L_mt = L_mt;               % 总行程之和
report.Threat1 = Threat1;           % 受威胁的航迹点位置  若穿越了威胁区 则不为0
report.Threat2 = Threat2;           % 受威胁的航迹点位置  若穿越了威胁区 则不为0
report.Threat1_D = Threat1_D; 
report.Threat2_D = Threat2_D; 
report.AngleProb = Angle;         % 转角不满足的点
report.TrajProb = MiniTraj;       % 不满足最小航迹间隔的点
report.ProbPoint = ProbPoint;     % 有问题的点
report.L = totalL;                % 飞行距离
report.time = totalTime;          % 飞行时间
report.col_times = CollideTimes;  % 碰撞次数

end



%% 穿越威胁区检测
function [across, across_num,RR] = CheckThreat(P1, P2, M)
    % 威胁区（球或圆区域，不适合圆柱区域）
    O = M(:,1:end-1);     % 圆心
    R = M(:, end);        % 半径 
    
    % 检测线段是否穿过某个障碍区
    total = 0;
    for i = 1 : size(O, 1)  % 对于每个威胁物
        % 计算航迹点之间、航迹点和威胁物中心之间的距离
         a = norm(P1 - P2);
         b = norm(P2 - O(i, :));
         c = norm(P1 - O(i, :));
         % P1点是否在圆内
         if c < R(i)         
             isHit = true;   
             RR(i)=c;
         % P2点是否在圆内
         elseif b < R(i)  
             RR(i)=b;
             isHit = true;   
         % P1 P2都不在圆内 就判断航线是否和威胁区域相交
         else
             PP = P2 - P1;
             PO = O(i, :) - P1;
             d = norm(cross(PP, PO)) / a;
             % 距离判据
             if d >= R(i)
                 isHit = false;  % 不相交
                 RR(i)=0;
             % 角度判据(距离满足条件时两个角都为锐角时相交)
             elseif d > 0
                 cosP1 = a^2 + c^2 - b^2 / (2*c*a);
                 cosP2 = a^2 + b^2 - c^2 / (2*b*a);
                 if cosP1 > 0  &&  cosP2 > 0
                     isHit = true;
                     RR(i)=d;
                 else
                     isHit = false;
                     RR(i)=0;
                 end
             % 两点在外，距离为0情况（共线情况）
             else
                 if a > b  &&  a > c
                    isHit = true;
                    RR(i)=d;
                 else
                    isHit = false;
                    RR(i)=0;
                 end
             end
%              RR=d;
         end

         if isHit
            total = total + 1; % 计数：该航迹穿越威胁区1次
         end
    end
    % 判断是否穿越威胁区
    if total > 0
        across = true;
    else
        across = false;   
    end
    across_num = total;     % 穿过威胁区的次数
end



%% 碰撞检测
function [collide] = CheckCollide(P1, P2, d_safe)
    if norm(P1 - P2) >= d_safe
        collide = false;
    else
        collide = true;
    end
end

