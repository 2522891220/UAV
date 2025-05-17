function Draw_example(UAV)

close all

map_select = UAV.Choose(1);
map_complexity =  UAV.Choose(2);

%% 确定威胁物、火炮点、禁飞区、起点/终点 绘图区

figure
%% 地图与威胁物、火炮点、禁飞区


H = UAV.H;
MAPSIZE_X = size(H,2); % x index: columns of H
MAPSIZE_Y = size(H,1); % y index: rows of H

% 火炮和雷达
threats_radar = UAV.Menace.radar;
threats_other = UAV.Menace.other;

% 禁飞区
NFZ = UAV.NFZ;
NFZ_num = size(NFZ,1);

% 起点/终点
p1= scatter3(UAV.Start(1,1),UAV.Start(1,2),UAV.Start(1,3),100,'bs','MarkerFaceColor','y');   hold on

p2= scatter3(UAV.Goal(1,1),UAV.Goal(1,2),UAV.Goal(1,3),100,'kp','MarkerFaceColor','y');
p3= scatter3(UAV.Start(2,1),UAV.Start(2,2),UAV.Start(2,3),100,'bs','MarkerFaceColor','y');
p4= scatter3(UAV.Goal(2,1),UAV.Goal(2,2),UAV.Goal(2,3),100,'kp','MarkerFaceColor','y');
p5= scatter3(UAV.Start(3,1),UAV.Start(3,2),UAV.Start(3,3),100,'bs','MarkerFaceColor','y');
p6= scatter3(UAV.Goal(3,1),UAV.Goal(3,2),UAV.Goal(3,3),100,'kp','MarkerFaceColor','y');
legend([p1,p2,p3,p4,p5,p6],{'起点1','终点1','起点2','终点2','起点3','终点3'})  ;      hold on



[X,Y] = meshgrid(1:MAPSIZE_X,1:MAPSIZE_Y);
s=mesh(X,Y,H); % Plot the data
s.FaceColor = 'flat';
colormap summer;                    % Default color map.
set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
axis equal vis3d on;            % Set aspect ratio and turn off axis.
shading interp;                  % Interpolate color across faces.
material dull;                   % Mountains aren't shiny.
camlight left;                   % Add a light over to the left somewhere.
lighting gouraud;                % Use decent lighting.
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
hold on

view(0,90)
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

h11=surf(x,y,z);
hold on
h11.EdgeColor = [.5, 0.18, .14];
h11.FaceColor = [0.69, 0.09, 0.12];
h11.FaceAlpha = 0.7;

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
h22=surf(x,y,z);
hold on
h22.EdgeColor = [0, 0, 0];
h22.FaceColor = [0.75, 0.75, 0.75];
h22.FaceAlpha = .7;

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
h33 = mesh(xc,yc,zc); % plot the cylinder
set(h33,'edgecolor',[0, 0, 0],'facecolor',[0.24,0.35,0.67],'FaceAlpha',.7); % set color and transparency

legend([h11,h22,h33],{'雷达威胁','火炮威胁','禁飞区'})


end



