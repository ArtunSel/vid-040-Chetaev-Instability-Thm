%% problem 1
clear all,close all,clc;
syms x1 x2;
f1=x1*x2-(x1^3)+(x2^3);f2=(x1^2)-(x2^2);
V=x1*x2;
Vdot=jacobian(V,[x1])*f1+jacobian(V,[x2])*f2;
Vdot=expand(Vdot)
char_V=char(V);
char_V=replace(char_V,'x1','x1_vec(ii)');
char_V=replace(char_V,'x2','x2_vec(jj)');
char_Vdot=char(Vdot);
char_Vdot=replace(char_Vdot,'x1','x1_vec(ii)');
char_Vdot=replace(char_Vdot,'x2','x2_vec(jj)');
x1_vec=linspace(-1,1,1e2);x2_vec=linspace(-1,1,1e2);
unstab_x1_vec=zeros(1,length(x1_vec)*length(x2_vec));
unstab_x2_vec=zeros(1,length(x1_vec)*length(x2_vec));
kk=1;
for ii=1:1:length(x1_vec)
    for jj=1:1:length(x2_vec)
        evalin('base',['V_temp','=',char_V,';']);
        evalin('base',['V_dot_temp','=',char_Vdot,';']);
        if V_temp>0 && V_dot_temp>0
            unstab_x1_vec(kk)=x1_vec(ii);
            unstab_x2_vec(kk)=x2_vec(jj);
            kk=kk+1;
        end
    end
end
plot([unstab_x1_vec],[unstab_x2_vec],'r.');hold on;
plot([0],[0],'o','MarkerSize',[10],'MarkerFaceColor','k');
%% ------------------------------------------------end of problem-1


%% problem 2
clear all,close all,clc;
syms x1 x2;
f1=x2;
f2=-x2-(x1^2);
V=x1*x2;
Vdot=jacobian(V,[x1])*f1+jacobian(V,[x2])*f2;
Vdot=expand(Vdot)
char_V=char(V);
char_V=replace(char_V,'x1','x1_vec(ii)');
char_V=replace(char_V,'x2','x2_vec(jj)');
char_Vdot=char(Vdot);
char_Vdot=replace(char_Vdot,'x1','x1_vec(ii)');
char_Vdot=replace(char_Vdot,'x2','x2_vec(jj)');
eps1=.001;
x1_vec=linspace(-eps1,eps1,1e2);
x2_vec=linspace(-eps1,eps1,1e2);
unstab_x1_vec=zeros(1,length(x1_vec)*length(x2_vec));
unstab_x2_vec=zeros(1,length(x1_vec)*length(x2_vec));
kk=1;
for ii=1:1:length(x1_vec)
    for jj=1:1:length(x2_vec)
        evalin('base',['V_temp','=',char_V,';']);
        evalin('base',['V_dot_temp','=',char_Vdot,';']);
        if V_temp>=0 && V_dot_temp>=0
            unstab_x1_vec(kk)=x1_vec(ii);
            unstab_x2_vec(kk)=x2_vec(jj);
            kk=kk+1;
        end
    end
end
plot([unstab_x1_vec],[unstab_x2_vec],'r.');hold on;
plot([0],[0],'o','MarkerSize',[10],'MarkerFaceColor','k');hold on;
%
x1_vec=linspace(-eps1,eps1,1e2);
x2_vec=linspace(-eps1,eps1,1e2);
unstab_x1_vec=zeros(1,length(x1_vec)*length(x2_vec));
unstab_x2_vec=zeros(1,length(x1_vec)*length(x2_vec));
kk=1;
for ii=1:1:length(x1_vec)
    for jj=1:1:length(x2_vec)
        evalin('base',['V_temp','=',char_V,';']);
        evalin('base',['V_dot_temp','=',char_Vdot,';']);
        if V_temp>=0 % && V_dot_temp>0
            unstab_x1_vec(kk)=x1_vec(ii);
            unstab_x2_vec(kk)=x2_vec(jj);
            kk=kk+1;
        end
    end
end
plot([unstab_x1_vec],[unstab_x2_vec],'bo');
axis square;
%% ------------------------------------------------end of problem-2


%% problem 3
clear all,close all,clc;
syms x1 x2;
f1=-x1-x1*x2;
f2=-(x2^3)-(x1^3);
% V=x1*x2;
% V=x1^2-x2^2;
V=x2^4-x1^4;
Vdot=jacobian(V,[x1])*f1+jacobian(V,[x2])*f2;
Vdot=expand(Vdot)
char_V=char(V);
char_V=replace(char_V,'x1','x1_vec(ii)');
char_V=replace(char_V,'x2','x2_vec(jj)');
char_Vdot=char(Vdot);
char_Vdot=replace(char_Vdot,'x1','x1_vec(ii)');
char_Vdot=replace(char_Vdot,'x2','x2_vec(jj)');
x1_vec=linspace(-.01,.01,1e2);
x2_vec=linspace(-.01,.01,1e2);
unstab_x1_vec=zeros(1,length(x1_vec)*length(x2_vec));
unstab_x2_vec=zeros(1,length(x1_vec)*length(x2_vec));
kk=1;
for ii=1:1:length(x1_vec)
    for jj=1:1:length(x2_vec)
        evalin('base',['V_temp','=',char_V,';']);
        evalin('base',['V_dot_temp','=',char_Vdot,';']);
        if V_temp>0 && V_dot_temp>0
            unstab_x1_vec(kk)=x1_vec(ii);
            unstab_x2_vec(kk)=x2_vec(jj);
            kk=kk+1;
        end
    end
end
plot([unstab_x1_vec],[unstab_x2_vec],'r.');hold on;
plot([0],[0],'o','MarkerSize',[10],'MarkerFaceColor','k');
%% ------------------------------------------------end of problem-3



%% let us look at some random trajectories
clear all,close all,clc;
fig1=figure(1);fig1.Color=[1,1,1];
ax1=axes('Parent',fig1);
    set(0,'CurrentFigure',fig1);
    set(fig1,'currentaxes',ax1);
for ii=1:1:3
    tspan=[0:0.01:2]; x0=1e-1*randi([-10,10],1,1)*[1;1];
%     x0=-1*abs(x0);
    wt=tspan;
    f=randi([1,10],1,1); w=sin(2*pi*f*tspan);
    [t,x]=ode45(@(t,x) odefcn(t,x,wt,w),tspan,x0);
    plot(x(:,1),x(:,2),'r-','LineWidth',[2],"Parent",ax1);
    hold on;
    plot(x(1,1),x(1,2),'ko','LineWidth',[2],"Parent",ax1);
    hold on;
end
plot([0],[0],'bo','LineWidth',[2],"Parent",ax1);axis square;
function xdot=odefcn(t,x,wt,w)
w=interp1(wt,w,t);
xdot=zeros(2,1);
x1=x(1);x2=x(2);
% % problem 1
% f1=x1*x2-(x1^3)+(x2^3);
% f2=(x1^2)-(x2^2);
% % problem 2
f1=x2;
f2=-x2-(x1^2);
% problem 3
% f1=-x1-x1*x2;
% f2=-(x2^3)-(x1^3);
xdot(1)=f1;
xdot(2)=f2;
end


%