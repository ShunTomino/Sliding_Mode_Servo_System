% 連続時間スライディングモードサーボ系を設計して、操舵による二輪車ロボットのロール角制御を行う。
clear; clc;

%% 二輪車ロボットのパラメータの設定
ToRad = pi/180; % deg -> rad 変換用
ToDeg = 180/pi; % rad -> deg 変換用
g = 9.8; % 重力加速度[m/s^2]

lf = 0.160; % 前輪から重心位置までの距離[m]
lr = 0.140; % 後輪から重心位置までの距離[m]
l = lf + lr; % 前輪から後輪までの距離[m]
m = 0.904; % 車両の質量[kg]
h = 0.155;  % 地面から質点までの高さ[m]
Ix = 0.0100; % 重心位置におけるロール方向の慣性モーメント[Kg・m^2]


%% 制御対象の状態空間モデル
% 状態変数:x = [phi; d_phi], 出力:y = phi (車体ロール角)
A = zeros(2,2);
B = zeros(2,1);
E = zeros(2,1);

A(1,2) = 1;
A(2,1) = m*g*h/(Ix+m*h^2);

B(2) = m*h/l/(Ix+m*h^2);

E(2) = -h/(Ix+m*h^2); 

C = [1,0];

%% 拡大系
% dx_/dt = A_*x_ + B_*u + G_*ref
% x_ = [w;x], w:目標ロール角との偏差の積分値

A_ = [0,-C;
    zeros(2,1),A]

B_ = [0;B]

G_ = [eye(1);0;0]

A11 = A_(1:2,1:2);
A12 = A_(1:2,3);

%% SMCの切替面設計
Q = diag([300,10]); % e,phiの重み
R = 1; % 入力の重み
F_ = lqr(A11,A12,Q,R);
S = [F_,eye(1)]

invSB_ = inv(S*B_);

%% 非線形入力項のパラメータ
k1 = 12;
k2 = 0.05;

%% Simulinkでシミュレーション実行
file = 'SMC_bicycle_sim';

open_system(file);
set_param(file,'WideLines','on');
set_param(file,'ShowLineDimensions','off');
sim(file);