function Opti_path_Plot_3(Track_all, UAV, Curve_name)

% close all;

%% 正视图

%% 路径转换
XXX = [];
YYY = [];
ZZZ = [];
for i=1:size(Track_all,1)
    Pos = SphericalToCart(Track_all(i,:),UAV);
    PointNum = UAV.PointNum(1);
    % Input solution
    XXX=[XXX; Pos.x];
    YYY=[YYY; Pos.y];
    ZZZ=[ZZZ; Pos.z];
end
% Start location
xs=UAV.Start(:,1);  % UAV.num 无人机数 UAVnum*1
ys=UAV.Start(:,2);
zs=UAV.Start(:,3);

% Final location
xf=UAV.Goal(:,1);
yf=UAV.Goal(:,2);
zf=UAV.Goal(:,3);

% 完整路径包含了起点和终点
x_all = [ XXX ];
y_all = [ YYY ];
z_all = [ ZZZ ];


v = Pos.v;

UAVnum = UAV.num;                        % 无人机个数
H = UAV.H;
dim = UAV.PointDim;                      % 仿真维度
track_num = size(Track_all,1);
for ii=1:track_num
    a.V = v(1, :)';    % 该个体的速度 转置成[UAVnum 1]
    %     P_a = P(1, :);     % 该个体的航迹点矩阵
    a.P = cell(UAVnum, 1);  % a.P含有UAVnum个元胞 用来保存每个UAV的航迹点坐标值，行为xyz 列为航迹点
    now_x = x_all(track_num*(ii-1)+1:track_num*(ii-1)+4,:);
    now_y = y_all(track_num*(ii-1)+1:track_num*(ii-1)+4,:);
    now_z = z_all(track_num*(ii-1)+1:track_num*(ii-1)+4,:);
    for i =1:UAVnum
        P_ai=[now_x(i,:); now_y(i,:); now_z(i,:)];% 这个UAV的轨迹
        % 真实高度
%         for iii = 1:PointNum
%             P_ai(3,iii) = P_ai(3,iii) + H(floor(P_ai(2,iii)),floor(P_ai(1,iii)));
%         end
        new_z(i,:) = P_ai(3,:);
    end
    XX = [];
    YY = [];
    ZZ = [];
    for i=1:UAVnum
        path = B_spline(now_x(i,:),now_y(i,:),new_z(i,:));
        XX(i,:) = path(1,:);
        YY(i,:) = path(2,:);
        ZZ(i,:) = path(3,:);
    end
    XXX(1:UAVnum,:) = [];
    YYY(1:UAVnum,:) = [];
    ZZZ(1:UAVnum,:) = [];
    % 保存了n种算法的m个无人机的xyz路线，共m*n个行
    X((ii-1)*UAVnum+1:(ii-1)*UAVnum+UAVnum, :) = XX;
    Y((ii-1)*UAVnum+1:(ii-1)*UAVnum+UAVnum, :) = YY;
    Z((ii-1)*UAVnum+1:(ii-1)*UAVnum+UAVnum, :) = ZZ;
end


H = UAV.H;
MAPSIZE_X = size(H,2); % x index: columns of H
MAPSIZE_Y = size(H,1); % y index: rows of H

% 火炮和雷达
threats_radar = UAV.Menace.radar;
threats_other = UAV.Menace.other;

% 禁飞区
NFZ = UAV.NFZ;
NFZ_num = size(NFZ,1);

% 如果无人机数量修改，绘图程序需要修改
% 航迹图
figure
set(gcf,'position',[250 300 1000 400])
% title('1')
%     subplot(1,track_num,j)
colors = {
    [0, 0.75, 0.75],  % 青色
    [0, 0, 0],        % 黑色
    [1, 0, 0],        % 红色
    [0.75, 0, 0.75],  % 紫色
    [0, 0, 1],        % 蓝色
    [0, 0.5, 0],      % 绿色
    [0.93, 0.69, 0.13],% 橙色
    [0, 0, 0],        % 黑色
    [0.5, 0, 0.5],    % 粉色
    [0, 1, 0],        % 绿色
    [1, 0.5, 0],      % 橙色
    [0.75, 0.75, 0]   % 棕色
    };

for j=1:track_num
    subplot(1,track_num,j)
    if j==1
        set(gca,'position', [0.05 0.1 0.3 0.9]);
    elseif j==2
        set(gca,'position', [0.36 0.1 0.3 0.9]);
    else
        set(gca,'position',[0.67 0.1 0.3 0.9]);
    end
    ii=1;
    h11 = plot3([UAV.Start(ii,1) X((j-1)*UAVnum+ii,:) UAV.Goal(ii,1)],...
        [UAV.Start(ii,2) Y((j-1)*UAVnum+ii,:) UAV.Goal(ii,2)],...
        [ UAV.Start(ii,3) Z((j-1)*UAVnum+ii,:) UAV.Goal(ii,3)], 'Color', colors{1}, 'LineWidth', 2);hold on   % 修改这里
    hold on
    ii=2;
    h22 = plot3([UAV.Start(ii,1) X((j-1)*UAVnum+ii,:) UAV.Goal(ii,1)],...
        [UAV.Start(ii,2) Y((j-1)*UAVnum+ii,:) UAV.Goal(ii,2)],...
        [ UAV.Start(ii,3) Z((j-1)*UAVnum+ii,:) UAV.Goal(ii,3)], 'Color', colors{4}, 'LineWidth', 2);hold on   % 修改这里
    hold on
    ii=3;
    h33 = plot3([UAV.Start(ii,1) X((j-1)*UAVnum+ii,:) UAV.Goal(ii,1)],...
        [UAV.Start(ii,2) Y((j-1)*UAVnum+ii,:) UAV.Goal(ii,2)],...
        [ UAV.Start(ii,3) Z((j-1)*UAVnum+ii,:) UAV.Goal(ii,3)], 'Color', colors{7}, 'LineWidth', 2);hold on   % 修改这里
    hold on


    ii=4;
    h44 = plot3([UAV.Start(ii,1) X((j-1)*UAVnum+ii,:) UAV.Goal(ii,1)],...
        [UAV.Start(ii,2) Y((j-1)*UAVnum+ii,:) UAV.Goal(ii,2)],...
        [ UAV.Start(ii,3) Z((j-1)*UAVnum+ii,:) UAV.Goal(ii,3)], 'Color', colors{5}, 'LineWidth', 2);hold on   % 修改颜色为 colors{5}
    hold on


    subtitle(Curve_name(j),'FontSize',12,'FontName','Times New Roman')


    [Xx,Yy] = meshgrid(1:MAPSIZE_X,1:MAPSIZE_Y);
    s=mesh(Xx,Yy,H); % Plot the data
    s.FaceColor = 'flat';
    colormap summer;                    % Default color map.
    %     set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
    axis equal vis3d on;            % Set aspect ratio and turn off axis.
    shading interp;                  % Interpolate color across faces.
    material dull;                   % Mountains aren't shiny.
    camlight left;                   % Add a light over to the left somewhere.
    lighting gouraud;                % Use decent lighting.
    xlabel('x [m]','FontSize',12,'FontName','Times New Roman');
    ylabel('y [m]','FontSize',12,'FontName','Times New Roman');
    zlabel('z [m]','FontSize',12,'FontName','Times New Roman');
    zlim([0,max(max(Z))+100])
    hold on

    %% 修改视角 不同的视角可以从这里修改 俯视图view(0,90) 侧视图view(-90,0) 正视图view(0,0)
    view(-20,20)
    box on
    grid on

    % 火炮
    for i = 1:size(threats_other,1)
        a = threats_other(i,1);
        b = threats_other(i,2);
        c = threats_other(i,3);
        R = threats_other(i,4);
        % 生成数据
        [x,y,z] = sphere(15);
        % 调整半径
        x = R*x;
        y = R*y;
        z = R*z;
        % 调整球心
        x = x+a;
        y = y+b;
        z = z+c;
        h1=surf(x,y,z);
        hold on
        h1.EdgeColor = [.5, 0.18, .14];
        h1.FaceColor = [0.69, 0.09, 0.12];
        h1.FaceAlpha = 0.7;
    end

    % 雷达
    for i =1:size(threats_radar,1)
        a = threats_radar(i,1);
        b = threats_radar(i,2);
        c = threats_radar(i,3);
        R = threats_radar(i,4);
        % 生成数据
        [x,y,z] = sphere(15);
        % 调整半径
        x = R*x;
        y = R*y;
        z = R*z;
        % 调整球心
        x = x+a;
        y = y+b;
        z = z+c;
        h2=surf(x,y,z);
        hold on
        h2.EdgeColor = [0, 0, 0];
        h2.FaceColor = [0.75, 0.75, 0.75];
        h2.FaceAlpha = .7;
    end


    % 禁飞区
    for i = 1:NFZ_num
        NFZ_x = NFZ(i,1);
        NFZ_y = NFZ(i,2);
        NFZ_z = NFZ(i,3);
        NFZ_radius = NFZ(i,4);
        h=H(NFZ_y,NFZ_x);

        [xc,yc,zc]=cylinder(NFZ_radius); % create a unit cylinder
        % set the center and height
        xc=xc+NFZ_x;
        yc=yc+NFZ_y;
        zc=zc*150+h;
        h3 = mesh(xc,yc,zc); % plot the cylinder
        set(h3,'edgecolor',[0, 0, 0],'facecolor',[0.24,0.35,0.67],'FaceAlpha',.7); % set color and transparency
    end

    % 起点/终点
    p1= scatter3(UAV.Start(1,1),UAV.Start(1,2),UAV.Start(1,3),100,'bs','MarkerFaceColor','y');   hold on

    p2= scatter3(UAV.Goal(1,1),UAV.Goal(1,2),UAV.Goal(1,3),100,'kp','MarkerFaceColor','y');
    scatter3(UAV.Start(2,1),UAV.Start(2,2),UAV.Start(2,3),100,'bs','MarkerFaceColor','y');
    scatter3(UAV.Goal(2,1),UAV.Goal(2,2),UAV.Goal(2,3),100,'kp','MarkerFaceColor','y');
    scatter3(UAV.Start(3,1),UAV.Start(3,2),UAV.Start(3,3),100,'bs','MarkerFaceColor','y');
    scatter3(UAV.Goal(3,1),UAV.Goal(3,2),UAV.Goal(3,3),100,'kp','MarkerFaceColor','y');
    % 调试：检查UAV4终点坐标
fprintf('UAV4终点坐标: (%.2f, %.2f, %.2f)\n', UAV.Goal(4,1), UAV.Goal(4,2), UAV.Goal(4,3));
fprintf('当前视图范围: zlim = [%.2f, %.2f]\n', 0, max(max(Z))+50);
    scatter3(UAV.Start(4,1),UAV.Start(4,2),UAV.Start(4,3),100,'bs','MarkerFaceColor','y');
    scatter3(UAV.Goal(4,1),UAV.Goal(4,2),UAV.Goal(4,3),100,'kp','MarkerFaceColor','y');
    if j==3
        legend([p1,p2,h11,h22,h33,h44],{'Start','End','UAV-1','UAV-2','UAV-3','UAV-4'},'FontSize',12,'FontName','Times New Roman')  ;      hold on
    end
    clear h11 h22 h33 h44
end
% title('Multi-UAV Path Planning','FontSize',12,'FontName','Times New Roman')

end
