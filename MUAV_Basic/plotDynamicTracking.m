% DynamicTargetUpdate.m
function [new_goal_pos] = DynamicTargetUpdate(initial_pos, velocity, time)
    % 更新动态目标的位置
    % initial_pos: 初始位置 [x,y,z]
    % velocity: 速度标量 (km/h)
    % time: 当前时间 (s)
    
    velocity_ms = velocity * 1000 / 3600; % 将km/h转换为m/s
    % 假设目标在x方向运动
    direction = [1, 0, 0] / sqrt(2);
    displacement = velocity_ms * time * direction;
    new_goal_pos = initial_pos + displacement;
end

% CalculateCaptureTime.m
function capture_time = CalculateCaptureTime(uav_pos, target_pos, target_velocity, uav_velocity)
    % 计算预计的捕获时间
    % 使用简化的追踪模型
    relative_pos = target_pos - uav_pos;
    distance = norm(relative_pos);
    
    % 假设UAV的速度大于目标速度
    if uav_velocity > target_velocity
        capture_time = distance / (uav_velocity - target_velocity);
    else
        capture_time = inf; % 如果UAV速度小于目标，将无法捕获
    end
end

% ModifiedFitnessFunction.m
function fitness = ModifiedFitnessFunction(solution, UAV, is_dynamic_target)
    % 修改后的适应度函数，考虑动态目标
    fitness = 0;
    
    % 对每个UAV计算适应度
    for i = 1:UAV.num
        if i == UAV.num && is_dynamic_target
            % 最后一个UAV追踪动态目标
            target_pos = DynamicTargetUpdate(UAV.Goal(i,:), 50, time);
            % 计算与动态目标的距离
            dist = norm(solution(end,:) - target_pos);
            fitness = fitness + dist;
        else
            % 静态目标的原有适应度计算
            dist = norm(solution(i,:) - UAV.Goal(i,:));
            fitness = fitness + dist;
        end
    end
end