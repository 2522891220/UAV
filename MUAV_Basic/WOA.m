

function [Convergence_curve , Leader_score, Leader_pos] = WOA(lb,ub,dim,fobj,Popsize,maxgen)




%% 种群初始化
fitness = zeros(1,Popsize); % 适应度
Positions = zeros(Popsize,dim); % 位置
for i = 1:Popsize
    Positions(i,:) = (ub - lb).*rand(1,dim) + lb;
    fitness(i) = fobj(Positions(i,:));
end
[SortFitness,indexSort] = sort(fitness); % 升序 第一个是最小的
% 最优个体
Leader_pos = Positions(indexSort(1),:);
Leader_score = SortFitness(1);

Convergence_curve=zeros(1,maxgen);%收敛曲线

%% 迭代优化
for t=1:maxgen

    % 控制参数a
    a=2-t*((2)/maxgen);
    % 更新l
    a2=-1+t*((-1)/maxgen);

    % 参数更新
    for i=1:size(Positions,1)
        % A和C更新
        r1=rand();
        r2=rand();
        A=2*a*r1-a;
        C=2*r2;
        % b和l更新
        b=1;
        l=(a2-1)*rand+1;

        % 更新个体
        for j=1:size(Positions,2)
            % 随机数p
            p = rand();
            if p<0.5
                if abs(A)>=1
                    % 搜索觅食机制
                    rand_leader_index = randi([1 Popsize]);
                    X_rand = Positions(rand_leader_index, :);
                    D_X_rand=abs(C*X_rand(j)-Positions(i,j));
                    Positions(i,j)=X_rand(j)-A*D_X_rand;
                elseif abs(A)<1
                    % 收缩包围机制
                    D_Leader=abs(C*Leader_pos(j)-Positions(i,j));
                    Positions(i,j)=Leader_pos(j)-A*D_Leader;
                end
            elseif p>=0.5
                % 螺旋更新位置
                distance2Leader=abs(Leader_pos(j)-Positions(i,j));
                Positions(i,j)=distance2Leader*exp(b.*l).*cos(l.*2*pi)+Leader_pos(j);
            end
        end
    end

    % 越界规范
    for i=1:size(Positions,1)
        Flag4ub=Positions(i,:)>ub;
        Flag4lb=Positions(i,:)<lb;
        Positions(i,:)=(Positions(i,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;%超过最大值的设置成最大值，超过最小值的设置成最小值
        fitness(i)=fobj(Positions(i,:));
        % 最优更新
        if fitness(i)<Leader_score
            Leader_score=fitness(i);
            Leader_pos=Positions(i,:);
        end
    end

    Convergence_curve(t)=Leader_score;
end


