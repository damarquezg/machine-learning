x = 0.1; % initial state
Q = 1; % process noise covariance
R = 1; % measurement noise covariance
tf = 50; % simulation length
N = 100; % number of particles in the particle filter
 
xhat = x;
P = 2;
xhatPart = x;
 
% Initialize the particle filter. ????????xpart????????????
for i = 1 : N
    xpart(i) = x + sqrt(P) * randn;
end
xArr = [x];
xhatPartArr = [xhatPart];
close all;
 
for k = 1 : tf         %tf??????k??????????k??
    % System simulation
       % x?????k??????
    x = 0.5 * x + 25 * x / (1 + x^2) + 8 * cos(1.2*(k-1)) + sqrt(Q) * randn; %????(1)
    y = x^2 / 20 + sqrt(R) * randn;%????(2)
    % Particle filter ??100???????????????????????
    for i = 1 : N
        xpartminus(i) = 0.5 * xpart(i) + 25 * xpart(i) / (1 + xpart(i)^2) + 8 * cos(1.2*(k-1)) + sqrt(Q) * randn;
        ypart = xpartminus(i)^2 / 20;
        vhat = y - ypart; %???????
        q(i) = (1 / sqrt(R) / sqrt(2*pi)) * exp(-vhat^2 / 2 / R); %????????
    end
    % Normalize the likelihood of each a priori estimate.
    qsum = sum(q);
    for i = 1 : N
        q(i) = q(i) / qsum;%?????
    end
    % Resample.
    for i = 1 : N
        u = rand; % uniform random number between 0 and 1
        qtempsum = 0;
        for j = 1 : N
            qtempsum = qtempsum + q(j);
            if qtempsum >= u
                              %???????????????????????????
                xpart(i) = xpartminus(j);
                break;
            end
        end
    end
 
    % The particle filter estimate is the mean of the particles.
    xhatPart = mean(xpart);    %???????????? 
    % Plot the estimated pdf's at a specific time.
    if k == 20
        % Particle filter pdf
        pdf = zeros(81,1);
        for m = -40 : 40
            for i = 1 : N
                if (m <= xpart(i)) && (xpart(i) < m+1)
                                     %pdf???????????xpart(i)???[m, m+1)????
                    pdf(m+41) = pdf(m+41) + 1;
                end
            end
        end
        figure;
        m = -40 : 40;
        %??1??k==20??xpart(i)??????
        plot(m, pdf / N, 'r');
        hold;
        title('Estimated pdf at k=20');
        disp(['min, max xpart(i) at k = 20: ', num2str(min(xpart)), ', ', num2str(max(xpart))]);
    end
    % Save data in arrays for later plotting
    xArr = [xArr x]; 
    xhatPartArr = [xhatPartArr xhatPart];
end
 
t = 0 : tf;
figure;
plot(t, xArr, 'b.', t, xhatPartArr, 'g');  %??2??xArr????xhatPartArr??????
xlabel('time step'); ylabel('state');
legend('True state', 'Particle filter estimate');