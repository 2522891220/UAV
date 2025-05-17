%% 清理空间
clc
clear
close all

global VarMax

%% 地图及问题建模
% 无人机初始位置
startPos = [  50,  50, 150;
              20,  100,180;
              100, 20, 150;
              150, 60, 180  ];

% 目标初始位置（包括静态和动态目标）          
goalPos = [  290, 370, 300;    % 静态目标1
             300, 400, 300;    % 静态目标2
             320, 400, 300;    % 动态目标3
             310, 300, 350  ]; % 动态目标4

% 动态目标参数设置
target_velocity = 0.3; % km/s
movement_direction = [1, 0, 0]; % X方向移动

map_select = 2;
map_complexity = 2;

flight_num = 15*ones(4,1); % 4个无人机的航迹点数量
UAV = Uav_Envir_Set(map_select, map_complexity, startPos, goalPos, flight_num, target_velocity, movement_direction);

% 展示所选的作战环境
Draw_example(UAV);

%% 算法参数定义
% 直角坐标的xyz范围
VarMin.x = UAV.limt.x(1,1);           
VarMax.x = UAV.limt.x(1,2);           
VarMin.y = UAV.limt.y(1,1);           
VarMax.y = UAV.limt.y(1,2);           
VarMin.z = UAV.limt.z(1,1);           
VarMax.z = UAV.limt.z(1,2);                 

for i=1:UAV.num
    % 球形坐标范围
    VarMax.r(i) = 2*norm(UAV.Start(i,:)-UAV.Goal(i,:))/flight_num(i);
    VarMin.r(i) = 0;

    % 仰角（极角）
    AngleRange = pi/4; % Limit the angle range for better solutions
    VarMin.psi(i) = -AngleRange;
    VarMax.psi(i) = AngleRange;

    % 方位角
    dirVector = UAV.Goal(i,:) - UAV.Start(i,:);
    phi0 = atan2(dirVector(2),dirVector(1));
    VarMin.phi(i) = phi0 - AngleRange;
    VarMax.phi(i) = phi0 + AngleRange;
end

% 整合成熟悉的形式
[lb,ub,dim,fobj] = Get_Spherical_details(VarMax, VarMin, UAV, flight_num);

SearchAgents_no = 300; % 种群数量  
Max_iteration = 300;   % 迭代次数

%% 多算法求解
% 粒子群算法求解
[PSO_Curve, PSO_fitness, PSO_chorm] = PSO(lb,ub,dim,fobj,SearchAgents_no,Max_iteration);
disp ' ---  PSO 完成 ---'

%% 灰狼算法求解
[GWO_Curve, GWO_fitness, GWO_chorm] = GWO(lb,ub,dim,fobj,SearchAgents_no,Max_iteration);
disp ' ---  GWO 完成 ---'

%% 鲸鱼算法求解
[WOA_Curve, WOA_fitness, WOA_chorm] = WOA(lb,ub,dim,fobj,SearchAgents_no,Max_iteration);
disp ' ---  WOA 完成 ---'

%% 集总数据
Track_all = [PSO_chorm;GWO_chorm;WOA_chorm];
Iter_all = [PSO_Curve; GWO_Curve; WOA_Curve];
Curve_name = ["PSO","GWO","WOA"];

%% 绘图
% 建模图 侧视图、俯视图 正视图
% Opti_path_Plot_1(Track_all, UAV, Curve_name);
% Opti_path_Plot_2(Track_all, UAV, Curve_name);
Opti_path_Plot_3(Track_all, UAV, Curve_name);
% 迭代曲线
Opti_Curve_Plot(Iter_all,Curve_name);