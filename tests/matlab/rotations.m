% Proof that the prediction step works without having to average the sigma
% point attitudes
rng default  % For reproducibility

f = @(n) reshape(randperm(n^2),n,n);
A = f(3) * 0.001;
Sigma = A * A'

n = 3
alpha = 0.25;
beta = 2.0;
kappa = 0.1;
lambda = alpha^2 * (n + kappa) - n;


A = (n + lambda) * Sigma;
L = chol(A, 'lower')

w = [0.1; 1.25; -.25];
dt = 0.01;

dq = qmat(rexp(w * dt));

q_0 = [1; 0; 0; 0];
q_1 = rexp(L(:,1));
q_2 = rexp(-L(:,1));
q_3 = rexp(L(:,2));
q_4 = rexp(-L(:,2));
q_5 = rexp(L(:,3));
q_6 = rexp(-L(:,3));

tq_0 = dq*q_0;
tq_1 = dq*q_1;
tq_2 = dq*q_2;
tq_3 = dq*q_3;
tq_4 = dq*q_4;
tq_5 = dq*q_5;
tq_6 = dq*q_6;

err_0 = rlog(qmat(rinv(tq_0)) * tq_0);
err_1 = rlog(qmat(rinv(tq_0)) * tq_1);
err_2 = rlog(qmat(rinv(tq_0)) * tq_2);
err_3 = rlog(qmat(rinv(tq_0)) * tq_3);
err_4 = rlog(qmat(rinv(tq_0)) * tq_4);
err_5 = rlog(qmat(rinv(tq_0)) * tq_5);
err_6 = rlog(qmat(rinv(tq_0)) * tq_6);

pts = [[0; 0; 0] L(:,1) -L(:,1) L(:,2) -L(:,2) L(:,3) -L(:,3)]
tpts = [[0; 0; 0] err_1 err_2 err_3 err_4 err_5 err_6]

scatter3(pts(1,:), pts(2,:), pts(3,:), 'bo');
hold on
scatter3(tpts(1,:), tpts(2,:), tpts(3,:), 'rx');
hold off

function frinv = rinv(q)
    frinv = [q(1); -q(2:4)];
end

function frexp = rexp(v)
    theta = norm(v);
    frexp = [cos(theta); v / theta * sin(theta)];
end

function frlog = rlog(q)
    theta = acos(q(1));
    frlog = theta / asin(theta) * q(2:4);
end

function fqmat = qmat(q)
    fqmat = [q(1) -q(2) -q(3) -q(4); q(2) q(1) -q(4) q(3); q(3) q(4) q(1) -q(2); q(4) -q(3) q(2) q(1)];
end