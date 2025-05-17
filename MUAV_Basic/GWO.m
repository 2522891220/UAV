

function [IterCurve , gBestFitness ,gBest] = GWO(lb,ub,dim,fobj,Popsize,maxgen)



% 定义Alpha，Beta，Delta狼
Alpha_pos=zeros(1,dim);
Alpha_score=inf;

Beta_pos=zeros(1,dim);
Beta_score=inf;

Delta_pos=zeros(1,dim);
Delta_score=inf;

% 迭代曲线
IterCurve = zeros(1,maxgen);

%% 种群初始化
fitness = zeros(1,Popsize);
for i = 1:Popsize
    Positions(i,:) = (ub - lb).*rand(1,dim) + lb;
    fitness(i) = fobj(Positions(i,:));
end

%% 对适应度排序，找到Alpha，Beta，Delta狼
%寻找适应度最小的位置
[SortFitness,indexSort] = sort(fitness);
%记录适应度值和位置
Alpha_pos = Positions(indexSort(1),:);
Alpha_score = SortFitness(1);
Beta_pos = Positions(indexSort(2),:);
Beta_score = SortFitness(2);
Delta_pos = Positions(indexSort(3),:);
Delta_score = SortFitness(3);
gBest = Alpha_pos;
gBestFitness = Alpha_score;

%% 开始迭代
for t = 1:maxgen
    %计算a的值
    a=2-t*((2)/maxgen);
    for i = 1:Popsize
        for j = 1:dim
            %% 根据Alpha狼更新位置
            r1=rand(); %[0,1]随机数
            r2=rand(); % [0,1]随机数
            A1=2*a*r1-a; % 计算A1
            C1=2*r2; % 计算C1
            D_alpha=abs(C1*Alpha_pos(j)-Positions(i,j)); % 计算距离Alpha的距离
            X1=Alpha_pos(j)-A1*D_alpha; %位置更新

            %% 根据Beta狼更新位置
            r1=rand();%[0,1]随机数
            r2=rand();%[0,1]随机数
            A2=2*a*r1-a;% 计算A2
            C2=2*r2; % 计算C2
            D_beta=abs(C2*Beta_pos(j)-Positions(i,j));  % 计算距离Beta的距离
            X2=Beta_pos(j)-A2*D_beta; %位置更新

            %% 根据Delta狼更新位置
            r1=rand();%[0,1]随机数
            r2=rand();%[0,1]随机数
            A3=2*a*r1-a; % 计算A3
            C3=2*r2; % 计算C3
            D_delta=abs(C3*Delta_pos(j)-Positions(i,j)); % 计算距离Delta的距离
            X3=Delta_pos(j)-A3*D_delta; %位置更新
            %更新位置
            Positions(i,j)=(X1+X2+X3)/3;
        end
        %% 边界检查
        Positions(i,:) = BoundaryCheck(Positions(i,:),ub,lb,dim);
    end
    %计算适应度值
    for i = 1:Popsize
        fitness(i) = fobj(Positions(i,:));
        % 更新 Alpha, Beta,  Delta狼
        if  fitness(i)<Alpha_score
            Alpha_score= fitness(i); % 更新alpha狼
            Alpha_pos=Positions(i,:);
        end
        if  fitness(i)>Alpha_score &&  fitness(i)<Beta_score
            Beta_score= fitness(i); % 更新beta狼
            Beta_pos=Positions(i,:);
        end
        if  fitness(i)>Alpha_score &&  fitness(i)>Beta_score &&  fitness(i)<Delta_score
            Delta_score= fitness(i); %更新delta狼
            Delta_pos=Positions(i,:);
        end
    end
    gBest = Alpha_pos;
    gBestFitness = Alpha_score;
    IterCurve(t) = gBestFitness;
end

end

%% 边界检查函数
function [X] = BoundaryCheck(x,ub,lb,dim)
    %dim为数据的维度大小
    %x为输入数据，维度为[1,dim];
    %ub为数据上边界，维度为[1,dim]
    %lb为数据下边界，维度为[1,dim]
    for i = 1:dim
        if x(i)>ub(i)
           x(i) = ub(i); 
        end
        if x(i)<lb(i)
            x(i) = lb(i);
        end
    end
    X = x;
end
