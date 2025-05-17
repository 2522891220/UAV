function position = SphericalToCart(sol,UAV)
global VarMax

% 修改为4个无人机的速度参数
V = sol(end-3:end);  % 现在是4个速度值

for ii=1:UAV.num  % UAV.num现在是4
    n = UAV.PointNum(ii);
    
    % 计算当前UAV参数在sol中的起始位置
    start_idx = 1;
    for jj = 1:ii-1
        start_idx = start_idx + 3*UAV.PointNum(jj);
    end
    
    % Start location
    xs = UAV.Start(ii,1);
    ys = UAV.Start(ii,2);
    zs = UAV.Start(ii,3);
    
    % Solution in Spherical space
    r = sol(start_idx:start_idx+n-1) + VarMax.r(ii)/2;
    psi = sol(start_idx+n:start_idx+2*n-1);
    phi = sol(start_idx+2*n:start_idx+3*n-1);
    
    % First Cartesian coordinate
    x(1) = xs + r(1)*cos(psi(1))*sin(phi(1));
    % Check limits
    if x(1) > UAV.limt.x(ii,2)
        x(1) = UAV.limt.x(ii,2);
    end
    if x(1) < UAV.limt.x(ii,1)
        x(1) = UAV.limt.x(ii,1);
    end
    
    y(1) = ys + r(1)*cos(psi(1))*cos(phi(1));
    if y(1) > UAV.limt.y(ii,2)
        y(1) = UAV.limt.y(ii,2);
    end
    if y(1) < UAV.limt.y(ii,1)
        y(1) = UAV.limt.y(ii,1);
    end
    
    z(1) = zs + r(1)*sin(psi(1));
    if z(1) > UAV.limt.z(ii,2)
        z(1) = UAV.limt.z(ii,2);
    end
    if z(1) < UAV.limt.z(ii,1)
        z(1) = UAV.limt.z(ii,1);
    end
    
    % Next Cartesian coordinates
    for i = 2:n
        x(i) = x(i-1) + r(i)*cos(psi(i))*sin(phi(i));
        if x(i) > UAV.limt.x(ii,2)
            x(i) = UAV.limt.x(ii,2);
        end
        if x(i) < UAV.limt.x(ii,1)
            x(i) = UAV.limt.x(ii,1);
        end
        
        y(i) = y(i-1) + r(i)*cos(psi(i))*cos(phi(i));
        if y(i) > UAV.limt.y(ii,2)
            y(i) = UAV.limt.y(ii,2);
        end
        if y(i) < UAV.limt.y(ii,1)
            y(i) = UAV.limt.y(ii,1);
        end
        
        z(i) = z(i-1) + r(i)*sin(psi(i));
        if z(i) > UAV.limt.z(ii,2)
            z(i) = UAV.limt.z(ii,2);
        end
        if z(i) < UAV.limt.z(ii,1)
            z(i) = UAV.limt.z(ii,1);
        end
    end
    
    % 为动态目标的情况更新终点位置
    if ii > 2  % 第3和第4个UAV追踪动态目标
        % 计算预计截获时间
        dist = norm([x(end) - xs, y(end) - ys, z(end) - zs]);
        intercept_time = dist / UAV.limt.v(ii,2); % 使用最大速度估算
        
        % 更新终点位置考虑目标移动
        target_pos = UAV.Goal(ii,:) + UAV.target_velocity * intercept_time * UAV.movement_direction;
        
        % 调整最后一个点朝向预测的目标位置
        x(end) = target_pos(1);
        y(end) = target_pos(2);
        z(end) = target_pos(3);
    end
    
    position.x(ii,:) = x;
    position.y(ii,:) = y;
    position.z(ii,:) = z;
    
    clear x y z  % 清除临时变量，防止下一次循环使用错误的数组大小
end

position.v = V;
end