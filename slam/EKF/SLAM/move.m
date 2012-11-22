function [ro, RO_r, RO_n] = move(r, u, n, dt)
% dt is not used by this function.

%初步估计a应该是运动物体的运动方向（这个角度是前一刻的物体的运动的朝向角度）
a = r(3);
%初步估计dx应该是移动距离，有两部分组成，其中一个是控制因素造成的移动距离u(1),而另外的一部分则是扰动因素造成的移动距离n(1)
dx = u(1) + n(1);
%初步估计da应该是运动物体的转动角度（这个部分的角度是由控制因素和扰动因素造成的）
da = u(2) + n(2);

%角度偏差
ao = a + da;
dp = [dx;0];

%把角度的表示范围归于[-pi, pi]之间
if ao > pi
    ao = ao - 2*pi;
end
if ao < -pi
    ao = ao + 2*pi;
end

if nargout == 1
    %这个地方，没有引入da，只有引入dx
    to = fromFrame2D(r, dp);
else    
    [to, TO_r, TO_dp] = fromFrame2D(r, dp);
    AO_a  = 1;
    AO_da = 1;
    
    RO_r = [TO_r ; 0 0 AO_a];%机器人状态转换的协方差矩阵
    RO_n = [TO_dp(:,1) zeros(2,1) ; 0 AO_da];%扰动因素（附加控制变量）的协方差矩阵
end
ro = [to;ao];

end

function f()
%%
syms x y a dx da real
X = [x;y;a];
u = [dx;da];
[xo, XO_x, XO_u] = move(X, u, zeros(2,1));
simplify(XO_x - jacobian(xo,X))
simplify(XO_u - jacobian(xo,u))
end

