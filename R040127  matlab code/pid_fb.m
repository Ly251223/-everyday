%https://blog.csdn.net/kilotwo/article/details/79828201?spm=1001.2101.3001.6650.3&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-3.pc_relevant_default&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-3.pc_relevant_default&utm_relevant_index=6

clear, clc
close all

Gs = tf(2,conv([3,1],[2,1]));           %Gs是传递函数，conv是可以理解为因式分解(3s+1)(2s+1)
Kp = [0.5,2,5,10];                      %取不同的比例系数,类似数组

for m = 1:4
    sys = feedback(Kp(m)*Gs,1);         %feedback(G,H),（G,H需事先设定）。
    %其中G是传递函数，H为反馈函数，表示一个控制系统G，对其进行负反馈H（要求正反馈用-H）。
    % 这里前面写上比例环节与系统的串联，后面的1表示负反馈
    step(sys);                          %求阶跃响应，可以用形如step(feedback(G,H))
    hold on;
end

%% 微分控制（与比例控制同时使用）的传递函数G(s)=Kp(1+TdS)
% 输出与输入偏差的微分成比例，即与偏差的变化速度成比例。
Kp = 10;
Td = [0,0.4,1,4];
for m = 1:4
    G1 = tf([Kp*Td(m),Kp],[0,1]);            %这个地方要注意 (Kp*Td(m)*S+Kp)/1=Kp(1+TdS)
    sys = feedback(G1*Gs,1);                 %前面是微分环节与系统的串联，负反馈
    step(sys); hold on;
end

%% 积分控制（与比例控制同时使用）的传递函数为： G(s)=Kp(1+1/Ti⋅1/s)
% 输出与输入偏差的积分成比例，即与误差的积累成比例。
% 取不同的积分系数，绘制系统的单位阶跃响应曲线：
Kp=10;
Ti=[3,6,12,24];
for m = 1:4
    G1=tf([Kp, Kp/Ti(m)],[1,0]); %这里也是(KpS+Kp/Ti)/s
    sys= feedback(G1,Gs,1);
    step(sys);hold on
end
% 加入积分控制后，消除了系统稳态误差，但随着Ti值的增大，达到稳态的过渡时间也逐渐加长。
% 积分项对误差取决于时间的积分，随着时间的增加，积分项会增大。
% 这样，即使误差很小，积分项也会随着时间的增加而加大，它推动控制器的输出增大
% 使稳态误差进一步减小，直到等于零，但会使系统稳定性降低，过渡时间也加长。

%% 比例积分微分控制,即PID控制
% 传递函数为：
% G(s)=Kp(1+1/T⋅1/s+TdS)  取适当的比例、积分、微分系数，绘制系统的单位阶跃响应曲线：
Kp = 100;                 % 取固定比例系数
Ti = 2.2;
Td =7;

G1 = tf([Kp*Td,Kp,Kp/Ti],[0,1,0])    %(Kp*Td*S^2+Kp*s+Kp/Ti)/s=Kp(Tds+1+1/STi)
sys = feedback(G1*Gs,1);
step(sys);
hold on;