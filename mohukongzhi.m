% %水位模糊控制算法
% clear all;
% close all;
%  
% a = newfis('fuzzy tank');
% 
% a = addvar(a,'input','e',[-3,3]);
% a = addmf(a,'input',1,'NB','zmf',[-3,-1]);
% a =addmf(a,'input',1,'NS','trimf',[-3,-1,1]);
% a =addmf(a,'input',1,'ZO','trimf',[-2,0,2]);
% a =addmf(a,'input',1,'PS','trimf',[-1,1,3]);
% a = addmf(a,'input',1,'PB','smf',[1,3]);
%  
% a = addvar(a,'output','u',[-4,4]);
% a = addmf(a,'output',1,'NB','zmf',[-4,-2]);
% a =addmf(a,'output',1,'NS','trimf',[-4,-2,0]);
% a =addmf(a,'output',1,'ZO','trimf',[-2,0,2]);
% a =addmf(a,'output',1,'PS','trimf',[0,2,4]);
% a = addmf(a,'output',1,'PB','smf',[2,4]);
%  
% %建立模糊规则模糊规则矩阵rulelist的含义：模糊矩阵是由模糊规则转化而来，这里矩阵规模为5*4，
% % 矩阵第一列表示输入e(5个模糊集合PB/PS/ZO/NS/NB依次对应1-5)，矩阵第二列表示输出u(含义同理)
% % ，第三列为规则的权重weight，第四列为AND模糊运算(1对应AND，2对应OR)，如果是多输入多数出模糊控制器，
% rulelist=[1 1 1 1;
%          2 2 1 1;
%          3 3 1 1;
%          4 4 1 1;
%          5 5 1 1];%第一列是指系统的输入。每一列都包含一个数字，该数字表示该变量的隶属函数的索引。 
%              %      接下来的n列是指系统的输出。每一列都包含一个数字，该数字表示该变量的membership函数索引。
%              %      m + n + 1列包含要应用于规则的权重。权重必须是一个介于0和1之间的数字，通常为1
% %              如果规则的前提条件的模糊运算符是AND，则m + n + 2列包含1。如果模糊运算符为或，则它包含2
% a = addrule(a,rulelist);
%  
% %设置反模糊化算法setfis(a,'DefuzzMethod','mom');反模糊化方法不同最终输出控制量也不同。
% % mom 最大隶属度平均法
% % centroid 面积重心法
% % bisector 面积等分法
% % som 最大隶属度取小法
% % lom 最大隶属度去大法
% a1 = setfis(a,'DefuzzMethod','mom');
% writefis(a1,'tank');
% a2 = readfis('tank');
%  
% figure(1);
% plotfis(a2);
% figure(2);
% plotmf(a,'input',1);
% figure(3);
% plotmf(a,'output',1);
%  
%  
% showrule(a);
% ruleview('tank');
%  
% for i=1:1:7
%    e(i)=i-4;
%    Ulist(i)=evalfis([e(i)],a2);
% end
% Ulist = round(Ulist);  %对决策结果四舍五入取整
% % 三种方法：round（四舍五入）、ceil（向上取整）、floor（向下取整）
%  
% disp('------------------------------------------------------');
% disp('----------模糊控制表：e =[-3,3], u = [-4,4]-----------');
% disp('------------------------------------------------------');
% fprintf('| a  |');
% fprintf(' %d  |',e);
% fprintf('\n');
% fprintf('| u  |');
% fprintf(' %d  |',Ulist);
% fprintf('\n');
%% 
function evalParamout=mohukongzhi(GoaLDist,ObstacleDist)
a=newfis('fuzzf');                   %创建新的模糊推理系统
%输入1 %GoaLDist评价函数
a=addvar(a,'input','GoaLDist',[0,20]);                   
 %添加 e 的模糊语言变量
% a=addmf(a,'input',1,'PS','trapmf',[0,0,1,1.5]);          
%  %添加 e 的模糊语言变量的隶属度函数（梯型）域为1.5-3.5
%   %隶属度函数为三角形0-1.5
% a=addmf(a,'input',1,'PM','trapmf',[1.5,2,3,3.5]);      
% a=addmf(a,'input',1,'PB','trapmf',[3.35,3.6,4,4]); 

a=addmf(a,'input',1,'PS','trapmf',[0,0,1,5]);          
 %添加 e 的模糊语言变量的隶属度函数（梯型）域为1.5-3.5
  %隶属度函数为三角形0-1.5
a=addmf(a,'input',1,'PM','trapmf',[5,6,8,13]);      
a=addmf(a,'input',1,'PB','trapmf',[12,15,17,20]); 

%输入2;%ObstacleDist评价函数
a=addvar(a,'input','ObstacleDist',[0,5]);                   
 %添加 ec 的模糊语言变量
a=addmf(a,'input',2,'PS','trapmf',[0,0,1,1.2]); 
a=addmf(a,'input',2,'PM','trimf',[1.2,2,3]);
a=addmf(a,'input',2,'PB','trapmf',[2.5,3,4.5,5]);

%输出1障碍物距离评价函数参数
a=addvar(a,'output','CalcDistEval',[0,1]);                 
   %添加 CalcDistEval 的模糊语言变量
a=addmf(a,'output',1,'PS','trapmf',[0,0,0.1,0.2]); 
a=addmf(a,'output',1,'PM','trimf',[0.15,0.3,0.45]);
a=addmf(a,'output',1,'PB','trapmf',[0.4,0.45,0.5,0.5]); 

% 输出2 heading角度的评价函数计算
a=addvar(a,'output','heading',[0,1]);                 
   %添加  heading 的模糊语言变量
a=addmf(a,'output',2,'PS','trapmf',[0,0,0.1,0.15]); 
a=addmf(a,'output',2,'PM','trimf',[0.1,0.25,0.4]);
a=addmf(a,'output',2,'PB','trapmf',[0.35,0.4,0.5,0.5]); 

% 
% %输出3速度评价函数参数
a=addvar(a,'output','vel',[0,1]);                 
   %添加vel的模糊语言变量
a=addmf(a,'output',3,'PS','trapmf',[0,0,0.2,0.3]); 
a=addmf(a,'output',3,'PB','trapmf',[0.5,0.7,1,1]); 
a=addmf(a,'output',3,'PM','trimf',[0.2,0.4,0.6]);
rulelist=[ 1 1 3 2 1 1 1;             %编辑模糊规则，后俩个数分别是规则权重和AND OR选项
           1 2 3 2 1 1 1;
           1 3 3 1 1 1 1;
           2 1 1 3 1 1 1;             %编辑模糊规则，后俩个数分别是规则权重和AND OR选项
           2 2 2 2 1 1 1;
           2 3 2 1 2 1 1;
           3 1 1 3 1 1 1;
           3 2 2 2 2 1 1;
           3 3 3 1 3 1 1];
       %第一列是指系统的输入。每一列都包含一个数字，该数字表示该变量的隶属函数的索引。
       %  接下来的n列是指系统的输出。每一列都包含一个数字，该数字表示该变量的membership函数索引。
       %   m + n + 1列包含要应用于规则的权重。权重必须是一个介于0和1之间的数字，通常为1
       %  如果规则的前提条件的模糊运算符是AND，则m + n + 2列包含1。如果模糊运算符为或，则它包含2
a=addRule(a,rulelist);                %添加模糊规则函数
showrule(a)  ;                           %显示模糊规则函数
a=setfis(a,'DefuzzMethod','centroid');                  %设置解模糊方法
writeFIS(a,'fuzzf');                    
%保存模糊系统

a=readfis('fuzzf');   %从磁盘读出保存的模糊系统
disp('fuzzy Controller table:e=[0,4],ec=[0,2]');%显示矩阵和数组内容

%推理%设置反模糊化算法setfis(a,'DefuzzMethod','mom');反模糊化方法不同最终输出控制量也不同。
% mom 最大隶属度平均法
% centroid 面积重心法
% bisector 面积等分法
% som 最大隶属度取小法
% lom 最大隶属度去大法
evalParamout=zeros(7,7);                                   %全零矩阵
for i=1:7
       for j=1:7
          GoaLDist(i)=-4+i;
          ObstacleDist(j)=-4+j;
           evalParamout=evalfis([GoaLDist(i), ObstacleDist(j)],a);
%            Ulist(i,j)=evalfis([e(i),ec(j)],a2);    %完成模糊推理计算
       end
   end
%   Ulist=ceil(Ulist)                               %朝正无穷方向取整
%    Ulist=round(Ulist) ;          
   evalParamout=round(evalParamout) ;    
   evalParamout%朝正无穷方向取整
%    三种方法：round（四舍五入）、ceil（向上取整）、floor（向下取整）
%画出模糊系统
% figure(1); plotfis(a);  
aw = 8; bw = 6;
set(gcf,'Units','centimeters','Position',[0*aw 1+0*bw aw bw]);%设置单位，位置大小8*6
figure(2);plotmf(a,'input',1);
set(gca,'FontName','Times New Roman','FontSize',12)%
aw = 8; bw = 6;
set(gcf,'Units','centimeters','Position',[0*aw 1+0*bw aw bw]);%设置单位，位置大小8*6
figure(3);plotmf(a,'input',2);grid on;
aw = 8; bw = 6;
set(gcf,'Units','centimeters','Position',[0*aw 1+0*bw aw bw]);%设置单位，位置大小8*6
set(gca,'FontName','Times New Roman','FontSize',12)%
figure(4);plotmf(a,'output',1);grid on;
aw = 8; bw = 6;
set(gcf,'Units','centimeters','Position',[0*aw 1+0*bw aw bw]);%设置单位，位置大小8*6
set(gca,'FontName','Times New Roman','FontSize',12)%
figure(5);plotmf(a,'output',2);grid on;
aw = 8; bw = 6;
set(gcf,'Units','centimeters','Position',[0*aw 1+0*bw aw bw]);%设置单位，位置大小8*6
set(gca,'FontName','Times New Roman','FontSize',12)%
figure(6);plotmf(a,'output',3);grid on;
aw = 8; bw = 6;
set(gcf,'Units','centimeters','Position',[0*aw 1+0*bw aw bw]);%设置单位，位置大小8*6
set(gca,'FontName','Times New Roman','FontSize',12)%
end