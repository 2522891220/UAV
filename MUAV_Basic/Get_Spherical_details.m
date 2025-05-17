
function [lb,ub,dim,fobj] = Get_Spherical_details(VarMax, VarMin, UAV,flight_num)


fobj=@(x) CostFunction(x,UAV);

UAVnum = UAV.num;
alpha = 0.7; % 减小
ub = [];
lb = [];
for i=1:UAVnum
    ub = [ub alpha*[VarMax.r(i)*ones(1,flight_num(i))/2 ,VarMax.psi(i)*ones(1,flight_num(i)) ,VarMax.phi(i)*ones(1,flight_num(i))]];
    lb = [lb alpha*[-VarMax.r(i)*ones(1,flight_num(i))/2 ,VarMin.psi(i)*ones(1,flight_num(i)) ,VarMin.phi(i)*ones(1,flight_num(i))]];
end

ub = [ub UAV.limt.v(:,1)' ];
lb = [lb UAV.limt.v(:,2)' ];


dim = flight_num(i)*3*UAVnum+4;



