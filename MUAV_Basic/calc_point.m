%% caculate the point on the track with timestamp <t1, t2, t3> (uniform motion)
%   3 timestamps are t1, t2, t3
%   'values' is the result ¡ª¡ª B-Spline 

function [ dot ] = calc_point( values, t1, t2, t3, t )
    [~,n_values] = size(values);
    len_values = 0.0; len_tmp = 0.0;
    % caculate the total length of B-Spline
    for j = 1 : n_values-1
        len_values = len_values + sqrt((values(1,j)-values(1,j+1))*(values(1,j)-values(1,j+1)) + (values(2,j)-values(2,j+1))*(values(2,j)-values(2,j+1)) + (values(3,j)-values(3,j+1))*(values(3,j)-values(3,j+1)));
    end
    % caculate the position of point with timestamp 't'
    for j = 1 : n_values-1
        len_tmp = len_tmp + sqrt((values(1,j)-values(1,j+1))*(values(1,j)-values(1,j+1)) + (values(2,j)-values(2,j+1))*(values(2,j)-values(2,j+1)) + (values(3,j)-values(3,j+1))*(values(3,j)-values(3,j+1)));
        if (len_tmp / len_values) > ((t-t1) / (t3-t1))
            dot = [values(1,j), values(2,j), values(3,j)];
            break;
        end
    end
end

