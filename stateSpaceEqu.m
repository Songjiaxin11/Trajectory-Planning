function [z_next] = stateSpaceEqu(z, u, dt)
% 四轮全向底盘离散状态空间方程dZ_r
% 输入参数
% z: t时刻状的态变量 3*1
% u: t时刻控制输入 4*1
% dt: 离散时间步长

% 输出参数
% z_next: t+dt时刻的状态变量

% 车辆几何参数
alpha_ = [pi*3/4; -pi*3/4; -pi/4; pi/4]; % 车辆安装角度
r_ = [1; 1; 1; 1]; % 车轮距离旋转中心的距离

Aw = [cos(alpha_), sin(alpha_), r_];

theta = z(3);

R = [ cos(theta), sin(theta);
     -sin(theta), cos(theta)];

% 机器人状态变量的微分
dz_r = inv(Aw' * Aw) * Aw' * u;
V_r = dz_r(1:2);
angularVel = dz_r(3);

V_w = inv(R) * V_r;

z_next = [0;0;0];
z_next(1:2) = z(1:2) + V_w * dt;
z_next(3) = z(3) + angularVel * dt;

% 输出矩阵
% dz = G(z) * u;
Gz(3,:) = Aw(3, :);
Gz(1:2,:) = inv(R) * Aw(1:2, :);

Gz

end

