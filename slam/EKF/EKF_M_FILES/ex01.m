% define system
% x+ = x + u * dt + n
% y  = x + v

dt = 1;

F_x = 1;
F_u = dt;
F_n = 1;
H = 1;

Q = 1;
R = 1;

% simulated variables

X = 7;
u = 1;

% estimated variables

x = 0;
P = 1e4;

% trajectories
tt = 0:dt:40;
XX = zeros(1, size(tt,2));
xx = zeros(1, size(tt,2));
yy = zeros(1, size(tt,2));
PP = zeros(1, size(tt,2));

% perturbation levels
q = sqrt(Q);
r = sqrt(R);

% start loop
i = 1;
for t = tt

    % simulate

    %在matlab里，randn将会产生一个期望为0，方差为1的正太分布。
    %则，经过计算，可以得到的是  cov(q * randn) = q * q * 1 = Q
    n = q * randn;

    %模拟真实值
    X = F_x * X + F_u * u + F_n * n;
    
    %同样的， cov(r * randn) = r * r * 1 = R
    v = r * randn;

    %模拟测量值（加入了测量误差）
    y = H*X + v;
    
    % estimate - prediction

    %预测值（没有加入过程噪声）
    x = F_x * x + F_u * u;

    %预测协方差
    P = F_x * P * F_x' + F_n * Q * F_n';
    
    % correction


    e = H * x;
    E = H * P * H';
    
    %测量余量  H * X + v - H * x
    z = y - e;

    %测量余量协方差  R + H * P * H'
    Z = R + E;
    
    %卡尔曼增益
    K = P * H' * Z^-1;
    
    %更新状态估计
    x = x + K * z;

    %更新协方差估计
    P = P - K * H * P;
    
    % collect data
    XX(:,i) = X;
    xx(:,i) = x;
    yy(:,i) = y;
    PP(:,i) = diag(P);

    % update index
    i = i + 1;
end

% plot
plot(tt,XX,'r',tt,xx,'c',tt,yy,'b',tt,XX+3*sqrt(PP),'g',tt,XX-3*sqrt(PP),'g');
legend('truth','estimate','measurement','+/- 3 sigma')

