DoF = 15;
N = 2 * DoF;

mu = [0 0 0 1 4 -1.2 0.1 0.2 -0.05 0 0 0 0 0 0];

f = @(n) reshape(randperm(n^2),n,n);
A = f(DoF) * 0.001;
sigma = A*A'
rng default  % For reproducibility
R = mvnrnd(mu,sigma,N);
R(N+1, :) = R(1, :);
R(1, :) = mu;

figure
plot3(R(:,1),R(:,2),R(:,3),'ob')

mu = mean(R);
mu(1:3) = 0;
C = zeros(DoF);
for i = 1:(N+1)
    v = R(i,:);
    C = C + (1/(N+1))*(v-mu)'*(v-mu);
end
mu
C
pts = R'

% Generate more points using our new Covariance Matrix
R = mvnrnd(mu,C,N);
R(N+1, :) = mu;
hold on
plot3(R(:,1),R(:,2),R(:,3),'or')
hold off
