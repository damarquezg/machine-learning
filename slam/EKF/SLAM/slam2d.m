% SLAM algorithm

% I. INITIALIZE
%
%   0. System def.

% System noise
q = [0.01;0.005];
Q = diag(q.^2);
% Measurement noise
m = [.15; 1*pi/180];
M = diag(m.^2);
% randn('seed',1);

%
%   1. Simulator
%       R: robot pose u: control
%机器人状态
R = [0;-2.5;0];
%控制因素
u = [0.1;0.05];
%生成了演示上的52个点。
W = cloister(-4,4,-4,4);%W存储的信息是各个特征点的全局坐标（由x，y系坐标值表示）
y = zeros(2,size(W,2));%存储的信息是各个特征点的相对于机器人的坐标（由d，a值表示），其中d为距离机器人的距离，a为与机器人的角度）

%   2. Estimator
x = zeros(numel(R)+numel(W), 1);
P = zeros(numel(x),numel(x));
mapspace = 1:numel(x);
l = zeros(2, size(W,2));
%找出mapspace向量里的头三个元素，并且赋给r
r = find(mapspace,numel(R));
%将r对应的下标的元素赋值为0
mapspace(r) = 0;
%将r对应的下标的x的元素赋值为R
x(r)   = R;
%初始化协方差矩阵
P(r,r) = 0;

%   3. Graphics
mapFig = figure(1);
cla;
axis([-6 6 -6 6])
axis square
WG = line('parent',gca,...
    'linestyle','none',...
    'marker','o',...
    'color','r',...
    'xdata',W(1,:),...
    'ydata',W(2,:));
RG = line('parent',gca,...
    'marker','.',...
    'color','r',...
    'xdata',R(1),...
    'ydata',R(2));
rG = line('parent',gca,...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',x(r(1)),...
    'ydata',x(r(2)));

lG = line('parent',gca,...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',[],...
    'ydata',[]);

eG = zeros(1,size(W,2));
for i = 1:numel(eG)
    eG(i) = line(...
        'parent', gca,...
        'color','g',...
        'xdata',[],...
        'ydata',[]);
end

reG = line(...
    'parent', gca,...
    'color','m',...
    'xdata',[],...
    'ydata',[]);

% II. Temporal loop

for t = 1:1200
    
    % 1. Simulator
    %扰动因素  
    n = q.*randn(2,1);
    %移动物体，由move函数来实现
    R = move(R, u, zeros(2,1));
    %这部分目的在于模拟测量值，为后面计算卡尔曼增益提供数据支持。
    for lid = 1:size(W,2)
        v = m.*randn(2,1);
        y(:,lid) = project(R, W(:,lid)) + v;%模拟测量值 可以使用的数据是机器人状态和特征点lid
        %project的返回值是对应的特征点与机器人的距离和与机器人的角度
    end
    
    % 2. Filter
    %   a. Prediction
    %   CAUTION this is sub-optimal in CPU time
    [x(r), R_r, R_n] = move(x(r), u, n);
    P_rr = P(r,r);
    P(r,:) = R_r*P(r,:);
    P(:,r) = P(r,:)';
    P(r,r) = R_r*P_rr*R_r' + R_n*Q*R_n';
    
    %   b. correction
    %       i. known lmks
    lids = find(l(1,:));%我想知道的是这里的l里的值是哪里赋值的呢，很是疑惑，因为还没有看到赋值的地方。 l这个向量存储的是x状态里的特征点的信息（索引信息）
    %在这里，对l向量进行解释，需要理解的是这里的向量l是用来保存特征点的。若没有特征点，则就直接跳过这段for循环。
    for lid = lids
        % expectation
        [e, E_r, E_l] = project(x(r), x(l(:,lid)));
        E_rl = [E_r E_l];
        rl   = [r l(:,lid)'];
        E    = E_rl * P(rl,rl) * E_rl';
        
        % measurement
        yi = y(:,lid);
        
        % innovation
        z = yi - e;
        if z(2) > pi
            z(2) = z(2) - 2*pi;
        end
        if z(2) < -pi
            z(2) = z(2) + 2*pi;
        end
        Z = M + E;
        
        % Kalman gain
        K = P(:, rl) * E_rl' * Z^-1;
        
        % update
        x = x + K * z;
        P = P - K * Z * K';
    end
    
    %       ii. init new lmks
    % check lmk availability
    % 发现特征点，添加入l里面，l是一个可变长的向量，用来存储特征点的。
    lid = find(l(1,:)==0 , 1);
    if ~isempty(lid)
        s = find(mapspace, 2);
        if ~isempty(s)
            mapspace(s) = 0;
            l(:,lid) = s';
            % measurement
            yi = y(:,lid);
            %x向量里的特征点是存储特征点的全局坐标的   l存储的是特征点的索引  
            [x(l(:,lid)), L_r, L_y] = backProject(x(r), yi);%这个地方，是把相对于机器人的坐标转换到全局的坐标。
            P(s,:) = L_r * P(r,:);
            P(:,s) = P(s,:)';
            P(s,s) = L_r * P(r,r) * L_r' + L_y * M * L_y';
        end
    end
    
    % 3. Graphics
    
    set(RG, 'xdata', R(1), 'ydata', R(2));
    set(rG, 'xdata', x(r(1)), 'ydata', x(r(2)));
    lids = find(l(1,:));
    lx = x(l(1,lids));
    ly = x(l(2,lids));
    set(lG, 'xdata', lx, 'ydata', ly);
    for lid = lids
        le = x(l(:,lid));
        LE = P(l(:,lid),l(:,lid));
        [X,Y] = cov2elli(le,LE,3,16);
        set(eG(lid),'xdata',X,'ydata',Y);
    end
    if t > 1
        re = x(r(1:2));
        RE = P(r(1:2),r(1:2));
        [X,Y] = cov2elli(re,RE,3,16);
        set(reG,'xdata',X,'ydata',Y);
    end
    drawnow;
end









