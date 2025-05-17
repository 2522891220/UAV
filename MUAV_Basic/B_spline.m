%% SPLINE 
%   Given a set of points in 'points.txt' to represent a broken line
%   and generate the according B-Spline
% function spline(~)
% generate a broken line as an example and write it into 'points.txt'
function path = B_spline(x,y,z)
points(1,:) = x;
points(2,:) = y;
points(3,:) = z;
% function 
% plot3(x,y,z,'o',x,y,z,'b:');
% fp = fopen('points.txt','wt');
% for i = 1 : length(x)
%     fprintf(fp, '%f ', x(i));
% end
% fprintf(fp, '\n');
% for i = 1 : length(y)
%     fprintf(fp, '%f ', y(i));
% end
% fprintf(fp, '\n');
% for i = 1 : length(z)
%     fprintf(fp, '%f ', z(i));
% end
% fclose(fp);
%% deal with 3 points in 'points.txt' each time, then generate the B-Spline of them
% points = importdata('points.txt');
len = length(points(1,:));  % number of points
path  = [];
% hold on;
for i = 1 : len - 2 % deal with points with number i,i+1,i+2 each time and generate the B-Spline of the 3 points
    values = smooth([points(1,i) points(1,i+1) points(1,i+2); points(2,i) points(2,i+1) points(2,i+2); points(3,i) points(3,i+1) points(3,i+2)],3,100);
%     plot3(values(1,:), values(2,:), values(3,:), 'r');  % draw 3 points in the broken line
    dot = calc_point(values, 1, 2, 4, 3);  % calc_point() works in uniform motion
                                           % calcpoint() works in variable
                                           % motion (which can instead of calc_point)
%     plot3(dot(1), dot(2), dot(3), 'go');    % draw the caculated point
    path = [path,values];
end
% hold off;
% axis equal;
% grid on;
 end

