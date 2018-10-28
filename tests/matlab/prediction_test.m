DoF = 9;
N = 1 + 2 * DoF;

dt = 1;
% First step

prev_w = [0; 0; 0];
prev_a = [0; 0; -9.80665];
prev_w_b = zeros(3, 1);
prev_a_b = zeros(3, 1);

next_w = [0.23; -0.012; 0.89];
next_a = [2.0; -1.012; -9.72];
next_w_b = zeros(3, 1);
next_a_b = zeros(3, 1);


prev_att = eye(3)
prev_pos = [0; 0; 0]
prev_vel = [0; 0; 0]

next_att = zeros(3, 1);
next_pos = [0; 0; 0];
next_vel = [0; 0; 0];

prev_w_ = prev_w - prev_w_b;
next_w_ = next_w - next_w_b;
w = (prev_w_ + next_w_) / 2;
skew_w = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
next_att = prev_att * expm(skew_w * dt)

g = [0; 0; -9.80665];
prev_a_ = prev_att * (prev_a - prev_a_b) - g;
next_a_ = next_att * (next_a - next_a_b) - g;
a = (prev_a_ + next_a_) / 2;
next_vel = prev_vel + a * dt

v = (prev_vel + next_vel) / 2;
next_pos = prev_pos + v * dt

%  Second step
%{
prev_w = next_w;
prev_a = next_a;
prev_w_b = next_w_b;
prev_a_b = next_a_b;

next_w = [-0.23; 1.012; 0.09];
next_a = [0;  0; -20];
next_w_b = [0.001; 0.02; -0.25];
next_a_b = [0.2;  -.6; 0.089];

prev_w_ = prev_w - prev_w_b;
next_w_ = next_w - next_w_b;
w = (prev_w_ + next_w_) / 2;
skew_w = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
next_att = prev_att * expm(skew_w * dt)

g = [0; 0; -9.8];
prev_a_ = prev_att * (prev_a - prev_a_b) - g;
next_a_ = next_att * (next_a - next_a_b) - g;
a = (prev_a_ + next_a_) / 2;
next_vel = prev_vel + a * dt

v = (prev_vel + next_vel) / 2;
next_pos = prev_pos + v * dt
%}


