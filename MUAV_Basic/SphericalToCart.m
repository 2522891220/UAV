


% 球转直
function position = SphericalToCart(sol,UAV)


global VarMax

V = sol(end-2:end);

for ii=1:UAV.num
    n = UAV.PointNum(ii);
    % Start location
    xs = UAV.Start(ii,1);
    ys = UAV.Start(ii,2);
    zs = UAV.Start(ii,3);

    % Solution in Sperical space
    r = sol(1:n)+VarMax.r(ii)/2;% 1:n
    psi = sol(n+1:2*n);%n+1:2n
    phi = sol(2*n+1:3*n);%2n+1:3n
    sol = sol(3*n+1:end);

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

        % z(i) = z(i-1) + r(i)*cos(psi(i));
        z(i) = z(i-1) + r(i)*sin(psi(i));
        if z(i) > UAV.limt.z(ii,2)
            z(i) = UAV.limt.z(ii,2);
        end
        if z(i) < UAV.limt.z(ii,1)
            z(i) = UAV.limt.z(ii,1);
        end
    end

    position.x(ii,:) = x;
    position.y(ii,:) = y;
    position.z(ii,:) = z;
end
   position.v = V;
end