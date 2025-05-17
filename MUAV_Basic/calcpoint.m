%% caculate the point on the track with timestamp <t1, t2, t3> (variable motion)
%   3 timestamps are t1, t2, t3
%   'values' is the result ¡ª¡ª B-Spline 

function [ dot ] = calcpoint( values, t1, t2, t3, t )
    [~,n_values] = size(values);
    len_values = 0.0; len_tmp = 0.0;
    % caculate the total length of B-Spline
    for j = 1 : n_values-1
        len_values = len_values + sqrt((values(1,j)-values(1,j+1))*(values(1,j)-values(1,j+1)) + (values(2,j)-values(2,j+1))*(values(2,j)-values(2,j+1)) + (values(3,j)-values(3,j+1))*(values(3,j)-values(3,j+1)));
    end
    half_len_values = len_values / 2; % half length of the B-Spline
    if t <= t2
        sp = t1; base = t2 - t1; delta = 0; 
    else
        sp = t2; base = t3 - t2; delta = half_len_values;
    end
    
    dot = [values(1,n_values), values(2,n_values), values(3, n_values)]; % initialization
    
    % caculate the position of point with timestamp 't'
    for j = 1 : n_values-1
        len_tmp = len_tmp + sqrt((values(1,j)-values(1,j+1))*(values(1,j)-values(1,j+1)) + (values(2,j)-values(2,j+1))*(values(2,j)-values(2,j+1)) + (values(3,j)-values(3,j+1))*(values(3,j)-values(3,j+1)));
        if ((len_tmp - delta) / half_len_values) > ((t-sp) / base)
            dot = [values(1,j), values(2,j), values(3,j)];
            break;
        end
    end
    
end

