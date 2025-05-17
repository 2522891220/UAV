
function [yy , fitnesszbest, zbest]  = PSO(lb,ub,dim,fobj,Popsize,maxgen)
% 粒子群


%% 参数初始化
%粒子群算法中的两个参数
c1 = 3.49445;
c2 = 3.49445;
%粒子更新速度
Vmax=5;
Vmin=-5;

%% 产生初始粒子和速度
for i=1:Popsize
    %随机产生一个种群
    pop(i,:)=lb + (ub - lb) .* rand( 1, dim );   %初始种群
    V(i,:)=1.*rands(1,dim);  %初始化速度
    %计算适应度
    fitness(i)=fobj(pop(i,:));            %染色体的适应度

end

%找最好的适应度值
[bestfitness bestindex]=min(fitness);
zbest=pop(bestindex,:);     %全局最佳
gbest=pop;                  %个体最佳
fitnessgbest=fitness;       %个体最佳适应度值
fitnesszbest=bestfitness;   %全局最佳适应度值

%% 迭代寻优
for i=1:maxgen
    for j=1:Popsize

        %速度更新
        V(j,:) = V(j,:) + c1*rand*(gbest(j,:) - pop(j,:)) + c2*rand*(zbest - pop(j,:));
        V(j,find(V(j,:)>Vmax))=Vmax;
        V(j,find(V(j,:)<Vmin))=Vmin;

        %种群更新
        pop(j,:)=pop(j,:)+0.5*V(j,:);

        pop(j,find(pop(j,:)>ub))=ub(find(pop(j,:)>ub));
        pop(j,find(pop(j,:)<lb))=lb(find(pop(j,:)<lb));
        %适应度值
        fitness(j)=fobj(pop(j,:));            %染色体的适应度

        %个体最优更新
        if fitness(j) < fitnessgbest(j)
            gbest(j,:) = pop(j,:);
            fitnessgbest(j) = fitness(j);
        end

        %群体最优更新
        if fitness(j) < fitnesszbest
            zbest = pop(j,:);
            fitnesszbest = fitness(j);
        end

    end
    yy(i)=fitnesszbest;
end




