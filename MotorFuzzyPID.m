clc;clear;close all;

% Fungsi transfer Plant
Ts = 0.01;
J = 0.01;
b = 0.1;
Ke = 0.01;
Kt = 0.01;
R = 1;
L = 0.5;
syms s;
K = Ke;
num = K;
den = sym2poly((J*s+b)*(L*s+R)+K^2);
sys = tf(num,den);
Plant = c2d(sys,Ts,'zoh');
figure
step(Plant)
title('Respon Sistem Awal')

open_system('SimFuzzyPID')
open_system('SimFuzzyPID/Fuzzy PID')

% Mendesain kendali PID konvensional
open_system('SimFuzzyPID/PID')

C0 = pid(1,1,1,'Ts',Ts,'IF','B','DF','B'); % PID structure
C = pidtune(Plant,C0) % design PID
[Kp, Ki, Kd] = piddata(C); % Parameter PID

% Asumsikan sinyal referensi bernilai 1, sehingga max. error |e|=1 
% Rentang input |E| adalah [-10 10], sehingga atur |GE| = 10.

GE = 100;
GCE = GE*(Kp-sqrt(Kp^2-4*Ki*Kd))/2/Ki; % Kp = GCU * GCE + GU * GE
GCU = Ki/GE; % Ki = GCU * GE
GU = Kd/GCE; % Kd = GU * GCE

% Fuzzy inference system Sugeno:
FIS = newfis('FIS','FISType','sugeno');

% Fungsi keanggotaan input error |E|:
FIS = addvar(FIS,'input','E',[-100 100]); 
FIS = addmf(FIS,'input',1,'Negative','gaussmf',[70 -100]);
FIS = addmf(FIS,'input',1,'Positive','gaussmf',[70 100]);

% Fungsi keanggotaan input perubahan error |CE|:
FIS = addvar(FIS,'input','CE',[-100 100]); 
FIS = addmf(FIS,'input',2,'Negative','gaussmf',[70 -100]);
FIS = addmf(FIS,'input',2,'Positive','gaussmf',[70 100]);

% Fungsi keanggotaan output |u|:
FIS = addvar(FIS,'output','u',[-200 200]); 
FIS = addmf(FIS,'output',1,'Min','constant',-200);
FIS = addmf(FIS,'output',1,'Zero','constant',0);
FIS = addmf(FIS,'output',1,'Max','constant',200);

% Aturan Fuzzy
ruleList = [1 1 1 1 1;...   % If |E| is Negative and |CE| is Negative then |u| is -200 (MIN)
            1 2 2 1 1;...   % If |E| is Negative and |CE| is Positive then |u| is 0 (ZERO)
            2 1 2 1 1;...   % If |E| is Positive and |CE| is Negative then |u| is 0 (ZERO)
            2 2 3 1 1];     % If |E| is Positive and |CE| is Positive then |u| is 200 (MAX)
FIS = addrule(FIS,ruleList);

sim('SimFuzzyPID')
load('StepPID')
load('StepFP')
figure
plot(StepPID(1,1:401),StepPID(2,101:501))
hold on
plot(StepFP(1,1:401),StepFP(2,101:501))
hold off
title('Respon Sistem Setelah Dikendalikan')
legend('PID','Fuzzy-PID')