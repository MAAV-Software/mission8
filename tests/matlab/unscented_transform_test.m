alpha = 0.1;
beta = 2.0;
kappa = 0.1;

n = 15;
N = 2*n + 1;

lambda = alpha^2 * (n + kappa) - n;

% Yaml read test
w_m_0_yaml = lambda / (n + lambda)
w_c_0_yaml = lambda / (n + lambda) + (1 - alpha^2 + beta)
w_yaml = 1 / (2 * (n + lambda))

(2 * n) * w + w_m_0
(2 * n) * w + w_c_0

% Compute sigma points
alpha = 0.25;
beta = 2.0;
kappa = 0.0;

lambda = alpha^2 * (n + kappa) - n;

w_m_0 = lambda / (n + lambda);
w_c_0 = lambda / (n + lambda) + (1 - alpha^2 + beta);
w = 1 / (2 * (n + lambda));

mu = zeros(n, 1);

rng default  % For reproducibility
f = @(n) reshape(randperm(n^2),n,n);
A = f(n) * 0.0001;
Sigma = A*A'

A = (n + lambda) * Sigma;
L = chol(A, 'lower');
x = zeros(n,N);
x(:,1) = mu;
for i = 1:n
    x(:, 2*i) = mu + L(:,i);
    x(:, 2*i+1) = mu - L(:,i);
end
x



