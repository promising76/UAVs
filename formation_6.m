function [ ] = formation_6 ()
close all;                 % 4follower and 1 leader
countmax=2000;
% stop_goal=[100 100];
 stop_goal=[40 40];
%% 初始化 位置pose、速度V、加速度控制量control
% start=[-8*2.5 -4*2.5 0 0 0;%%%[x y th]
%     -11*2.5 -2*2.5 0 0 0;
%         -7*2.5 -7*2.5 0 0 0;
%     -7*2.5 -4*2.5 0 0 0;
%     -3*1.5 -2*1.5 0 0 0];
% obstacle=[ 20 -20*rand(1);
%     60 100*rand(1);
%     120 50*rand(1);
%     250 50;
%     150 100
%     200 200
%     175 175];
obstacleR=0.5;% 冲突判定用的障碍物半径
%% 初始化 位置pose、速度V、加速度控制量control
%         start=[-7 -4 0 0 0;%%%[x y th]
%                 -6 -2 0 0 0; 
%                 -10 -3 0 0 0;
%                 -6 -4 0 0 0;
%                 -3 -2 0 0 0];   
start=[3 1 0 0 0;%%%[x y th]
    0.5 0.5 0 0 0;
    1 1 0 0 0;
   0 1 0 0 0;
    5 5 0 0 0];

      % 模拟实验的结果
result1=[];
result2=[];
result3=[];
result4=[];
result5=[];
x1=start(1,:)';
x2=start(2,:)';
x3=start(3,:)';
x4=start(4,:)';
x5=start(5,:)';%x5为长机
% 评价函数参数 [heading,dist,velocity,predictDT]
% Main loop
for m=1:countmax
    obstacle=[  
%     15 0.05*m;
%     25 0.03*m;
%     15 17;
%     20 22
%     30 35
        50 50];
    %% 轨迹跟踪控制器
      for i=1:4  % DWA参数输入
    distance1=sqrt((start(5,1)-start(i,1))^2+((start(5,1)-start(i,1))^2));
      end
        if  distance1<5
%             Kinematic5=[30,20,8,0.2,0.01,toRadian(1)];
 Kinematic5=[1,20,0.2,0.2,0.01,toRadian(1)];
        else
%             Kinematic5=[10,20,1,0.15,0.01,toRadian(1)];
  Kinematic5=[0.5,10,0.15,0.15,0.01,toRadian(1)];
     distance2=sqrt((start(i,1)-start(i+1,1)^2+(start(i,2)-start(i,2))^2));
       end
     if  distance2<10
%         Kinematic=[60,20,10,0.4,0.01,toRadian(1)];
  Kinematic=[2,20,0.4,0.4,0.01,toRadian(1)];
     else
%          Kinematic=[50,20,12,0.4,0.1,toRadian(1)];
   Kinematic=[2,20,0.4,0.4,0.01,toRadian(1)];
    end%若距离过远 各机速度减慢
evalParam=[0.05,0.3,0.1,3.0]; %航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
    [u5,traj5]=DynamicWindowApproach(x5,Kinematic5,stop_goal,evalParam, obstacle,obstacleR);
     %% follower相对leader的位置
      if x5(1)<15
        goal1=[x5(1)-3  x5(2)];
        goal2=[x5(1)-6 x5(2)];
        goal3=[x5(1) x5(2)-3];
        goal4=[x5(1) x5(2)-6];
      else
          goal1=[x5(1)-3 x5(2)];
          goal2=[x5(1)-1.5 x5(2)-1.5];%长机僚机的相对位置
          goal3=[x5(1) x5(2)-3];
          goal4=[x5(1)-3 x5(2)-3];
  
    end
%     if x5(1)<50
%         goal1=[x5(1)-6*1.5  x5(2)-6*1.5];
%         goal2=[x5(1) x5(2)-6*1.5];
%         goal3=[x5(1)-6*1.5 x5(2)];
%         goal4=[x5(1)-3*1.5 x5(2)-3*1.5];
% else  
%         goal1=[x5(1)-6*1.5 x5(2)];%长机僚机的相对位置
%         goal2=[x5(1) x5(2)-6*1.5];
%         goal3=[x5(1)-3*1.5 x5(2)];
%         goal4=[x5(1) x5(2)-3*1.5];
% %     else
% %         goal1=[x5(1)-6 x5(2)-6];
% %         goal2=[x5(1)-4.5 x5(2)-4.5];
% %         goal3=[x5(1)-3 x5(2)-3];
% %         goal4=[x5(1)-1.5 x5(2)-1.5];
%     end
    %% 动态障碍物   
%        mo=[];
%       flag_obstacle = [1-2*rand(1) 1-2*rand(1) 1-2*rand(1)];
%     vel_obstacle =0.25;%动态障碍物
%      for j = 1:3
%         if obstacle(j,2) > 10 && flag_obstacle(j) > 0 || obstacle(j,2) < 0 && flag_obstacle(j) < 0
%             flag_obstacle(j) = -flag_obstacle(j);
%         end
%            obstacle(j,2)=obstacle(j,2)+flag_obstacle(j)*vel_obstacle;%动态障碍物
%      end
%      n=m-1;
%     for i=1:3
%         mo(i+3*n,:)=[obstacle(1,1),obstacle(1,2)];
%         mo(i+1+3*n,:)=[obstacle(2,1),obstacle(2,2)];
%         mo(i+2+3*n,:)= [obstacle(3,1),obstacle(3,2)];
%            save('mo.mat','mo')
%     end
    other_temp1=[x2(1) x2(2)
   x3(1) x3(2)
   x4(1) x4(2)
    obstacle(:,1) obstacle(:,2)];
  other_temp2=[x1(1) x1(2)
   x3(1) x3(2)
   x4(1) x4(2)
    obstacle(:,1) obstacle(:,2)];
  other_temp3=[x2(1) x2(2)
   x1(1) x1(2)
   x4(1) x4(2)
    obstacle(:,1) obstacle(:,2)];
  other_temp4=[x2(1) x2(2)
   x3(1) x3(2)
    obstacle(:,1) obstacle(:,2)
   x1(1) x1(2)
]; %将僚机作为动态障碍物进行避障
    [u4,traj4]=DynamicWindowApproach(x4,Kinematic,goal1,evalParam,other_temp4,obstacleR);
    [u3,traj3]=DynamicWindowApproach(x3,Kinematic,goal2,evalParam,other_temp3,obstacleR);
    [u2,traj2]=DynamicWindowApproach(x2,Kinematic,goal3,evalParam,other_temp2,obstacleR);
    [u1,traj1]=DynamicWindowApproach(x1,Kinematic,goal4,evalParam,other_temp1,obstacleR);
    x1  =f(x1,u1);% 机器人移动到下一个时刻
    x2 =f(x2,u2);% 机器人移动到下一个时刻
    x3 =f(x3,u3);% 机器人移动到下一个时刻
    x4 =f(x4,u4);% 机器人移动到下一个时刻
    x5 =f(x5,u5);% 机器人移动到下一个时刻
    % 模拟结果的保存 
    result1=[ result1; x1'];
    result2=[ result2; x2'];
    result3=[ result3; x3'];
    result4=[ result4; x4'];
    result5=[ result5; x5'];
   %====Animation====% 探索轨迹


%          plot( obstacle(1,1), obstacle(1,2),'-k');
     aw = 8; bw = 6;
set(gcf,'Units','centimeters','Position',[0*aw 1+0*bw aw bw]);%设置单位，位置大小8*6 
xlabel('$x/m$','interpreter','latex');ylabel('$y/m$','interpreter','latex');
set(gca,'FontName','Times New Roman','FontSize',10)%设置坐标轴刻度字体名称，大小   
hold off;    
% draw_circle(obstacle(1,1),obstacle(1,2),0.5,'g');hold on
% draw_circle(obstacle(1,1),obstacle(1,2),0.5,'k');hold on
% draw_circle(obstacle(2,1),obstacle(2,2),0.5,'k');hold on;
% draw_circle(obstacle(3,1),obstacle(3,2),0.5,'g');hold on;
% draw_circle(obstacle(4,1),obstacle(4,2),0.5,'g');hold on;
% draw_circle(obstacle(5,1),obstacle(5,2),0.5,'k');hold on
            plot(result5(:,1),result5(:,2),'-r');hold on;%画出x的轨迹
            plot(result4(:,1),result4(:,2),'-b');hold on;%画出x的轨迹
            plot(result3(:,1),result3(:,2),'-b');hold on;%画出x的轨迹
            plot(result2(:,1),result2(:,2),'-b');hold on;%画出x的轨迹
            plot(result1(:,1),result1(:,2),'-b');hold on;%画出x的轨迹
            org = [0,0,0];
            [angle_x,angle_y,angle_z] = xiang_liang_to_ou_la([1,1,0],org);
            drawBody(x5(1),x5(2),60,angle_x,angle_y,angle_z,[]);
            drawBody(x4(1),x4(2),60,angle_x,angle_y,angle_z,[]);
            drawBody(x3(1),x3(2),60,angle_x,angle_y,angle_z,[]);
            drawBody(x2(1),x2(2),60,angle_x,angle_y,angle_z,[]);
            drawBody(x1(1),x1(2),60,angle_x,angle_y,angle_z,[]);%画飞机
        grid on;
     if ~isempty(traj5)
        for it=1:length(traj5(:,1))/5
            ind=1+(it-1)*5;
            plot(traj5(ind,:),traj5(ind+1,:),'-g');
        end
    end%画探索轨迹
        if norm(x5(1:2)-stop_goal')<0.5
            disp('Arrive Goal!!');
        end
    grid on;
    drawnow;
% save('x5.mat','result5')
% save('x4.mat','result4')
% save('x3.mat','result3')
% save('x2.mat','result2')
% save('x1.mat','result1')
end
end
function [ output_args ] = draw_circle (x,y,r,color)
%UNTITLED4 姝ゅ鏄剧ず鏈夊叧姝ゅ嚱鏁扮殑鎽樿
%   姝ゅ鏄剧ず璇︾粏璇存槑
t=0:0.01:2*pi;
X=x+r*cos(t);
Y=y+r*sin(t);
plot(X,Y,color);
end
function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)% DWA参数输入% Dynamic Window [vmin,vmax,wmin,wmax]
Vr=CalcDynamicWindow(x,model);% 评价函数的计算 evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
%               trajDB      每5行一条轨迹 每条轨迹都有状态x点串组成
% 评价函数的计算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);% model  机器人运动学模型  最高速度[m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss], 速度分辨率[m/s],转速分辨率[rad/s]]
% 输入参数：当前状态、模型参数、目标点、评价函数的参数、障碍物位置、障碍物半径
% 返回参数：控制量 u = [v(m/s),w(rad/s)] 和 轨迹集合 N * 31  （N：可用的轨迹数）
% 选取最优参数的物理意义：在局部导航过程中，使得机器人避开障碍物，朝着目标以较快的速度行驶。
if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];
    return;
end
    %% 新增控制器
% GoaLDist=norm(goal(1:2)-x(1:2));%新增目标到当前位置距离
% if GoaLDist>20
%     GoaLDist=20;
% end
% ObstacleDist=norm(ob(1:2)-x(1:2));%新增障碍物到当前位置距离
% if ObstacleDist>5
%    ObstacleDist=5;
% end
% evalParamout=[mohukongzhi(GoaLDist,ObstacleDist),0.1,3]; %航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
% eval=evalParamout(1:3);
% evalParam(1)=evalParam(1)+evalParamout(1);
% evalParam(2)=evalParam(1)+evalParamout(2);
% evalParam(3)=evalParam(1)+evalParamout(3);
% save('evalParam.mat','eval')
% 各评价函数正则化
evalDB=NormalizeEval(evalDB);
% 最终评价函数的计算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)']; %根据评价函数参数 前三个参数分配的权重 计算每一组可用的路径参数信息的得分
end  % 遍历各个可运行的路径，分别计算其评价得分
evalDB=[evalDB feval];% 最后一组；加最后一列，每一组速度的最终得分
[maxv,ind]=max(feval);% 最优评价函数)选取评分最高的参数 对应分数返回给 maxv  对应下标返回给 
u=evalDB(ind,1:2)';% 4ind 返回最优参数的速度、角速度  
end
function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)% 评价函数的计算
% 
evalDB=[];
trajDB=[];
for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        % 轨迹推测; 得到 xt: 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);  %evalParam(4),前向模拟时间;
        % 各评价函数的计算
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,ob,R);
        vel=abs(vt);
        % 制动距离的计算
        stopDist=CalcBreakingDist(vel,model);
        if dist>stopDist % 
            evalDB=[evalDB;[vt ot heading dist vel]];
            trajDB=[trajDB;traj];
        end
    end
end
end
function EvalDB=NormalizeEval(EvalDB)% 各评价函数正则化% 评价函数归一化
if sum(EvalDB(:,3))~=0 %航向得分
    EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));%矩阵的数除  单列矩阵的每元素分别除以本列所有数据的
end
if sum(EvalDB(:,4))~=0 % 距离得分
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
    if sum(EvalDB(:,5))~=0% 速度得分
        EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
    end
end
function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)% % 轨迹生成函数% 输入参数： 当前状态、vt当前速度、ot角速度、evaldt 前向模拟时间、机器人模型参数（没用到）返回 预测的x和到达该x所经过的若干点 （将后者依次连线，就可得到一条预测的轨迹）
%           x   : 机器人模拟时间内向前运动 预测的终点位姿(状态); 
%           traj: 当前时刻 到 预测时刻之间 过程中的位姿记录（状态记录） 当前模拟的轨迹  
%                  轨迹点的个数为 evaldt / dt + 1 = 3.0 / 0.1 + 1 = 31     
% evaldt：前向模拟时间; vt、ot当前速度和角速度; 
time=0;
u=[vt;ot];% 输入值
traj=x;% 机器人轨迹
while time<=evaldt
dt=0.1;
    time=time+dt;% 时间更新
    x=f(x,u);% 运动更新，前项模拟时间内 速度、角速度恒定
    traj=[traj x];% 每一列代表一个轨迹点 一列一列的添加
end
end
function stopDist=CalcBreakingDist(vel,model)%根据运动学模型计算制动距离, 也可以考虑成走一段段圆弧的累积 简化可以当一段段小直线的累积利用 当前速度和机器人可达到的加速度，计算其速度减到0所走距离  
stopDist=0;
dt=0.1;
while vel>0%给定加速度的条件下 速度减到0所走的距离
    stopDist=stopDist+vel*dt;% 制动距离的计算
    vel=vel-model(3)*dt;% 
end
end
%（机器人在当前轨迹上与最近的障碍物之间的距离，如果没有障碍物则设定一个常数）
% 输入参数：位姿、所有障碍物位置、障碍物半径
% 输出参数：当前预测的轨迹终点的位姿距离所有障碍物中最近的障碍物的距离 如果大于设定的最大值则等于最大值
% 距离障碍物距离越近分数越低
function dist=CalcDistEval(x,ob,R)% 障碍物距离评价函数
dist=100; % 无障碍物的默认值
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;
    if dist>disttmp% 离障碍物最小的距离
        dist=disttmp;
    end
end
% 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
if dist>=2*R
    dist=2*R;%最大分数限制
end
end
%% heading的评价函数计算   ok
% 输入参数：当前位置、目标位置
% 输出参数：航向参数得分 = 180 - 偏差值
% 当前车的航向和相对于目标点的航向 偏离程度越小 分数越高 最大180分
function heading=CalcHeadingEval(x,goal)% heading角度的评价函数计算
theta=toDegree(x(3));% 机器人朝向
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 目标点的方位
if goalTheta>theta
    targetTheta=goalTheta-theta;% [deg]
else
    targetTheta=theta-goalTheta;% [deg]
end
heading=180-targetTheta;
end
function Vr=CalcDynamicWindow(x,model)% 车子速度的最大最小范围
dt=0.1;
Vs=[0 model(1) -model(2) model(2)];
% 根据当前速度以及加速度限制计算的动态窗口
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
% 最终的Dynamic Window
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
end
function x = f(x, u)% Motion Model% u = [vt; wt];当前时刻的速度、角速度
dt=0.1;
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];
x= F*x+B*u;
end
function radian = toRadian(degree)%角度->弧度
% degree to radian
radian = degree/180*pi;
end
function degree = toDegree(radian)%弧度->角度
% radian to degree
degree = radian/pi*180;
end