% Trajectory Generation _ Obstacle Avoidance

clc,clear all,close all
warning off

t=1:0.1:10;
x0=-5;y0=-1;xf=10;yf=2;
y=((yf-y0)/(xf-x0))*(t-0)+y0;  % the straight line that connects the start and final points


a=6;b=2; %center of the obstacle
r=1; % the obstacle radius
xc=r.*sin(t)+a; 
c=r.*cos(t)+b;
hold on,plot(xc,c);axis equal, % plot the circular obstacle


r2=2*r; % radius for the safe region
x2=r2.*sin(t)+a;
c2=r2.*cos(t)+b;
hold on,plot(x2,c2);axis equal

y2=((b-y0)/(a-x0))*(t-x0)+y0; % the straight line joining the start point and the center of the circle center


syms m n
S=solve([(m-a)^2+(n-b)^2==r2^2,n==((b-y0)/(a-x0))*(m-x0)+y0],[m,n])
S=[S.m S.n]

if S(2,1)<S(1,1)
    x1=S(2,1);y1=S(2,2);
else
    x1=S(1,1);y1=S(1,2);
end
hold on,plot(x1,y1,'*b') % the closest point on the safe region from the start point

y3=((yf-b)/(xf-a))*(t-xf)+yf; % the straight line joining the final point and the center of the circle center


syms mc nc
Sc=solve([(mc-a)^2+(nc-b)^2==r2^2,nc==((yf-b)/(xf-a))*(mc-xf)+yf],[mc,nc])


Sc=[Sc.mc Sc.nc]


if Sc(2,1)>Sc(1,1)
    x2=Sc(2,1);y2=Sc(2,2);
else
    x2=Sc(1,1);y2=Sc(1,2);
end
hold on,plot(x2,y2,'*k') % the closest point on the safe region from the final point
title('Constraint-Objective Avoidance')

xc=(x1+x2)/2;yc=(y1+y2)/2;

yc2=b+r2;

% the position of the new point for obstacle avoidance

% New Trajectory
t2=1:0.1:xc;
ycm1=((yc2-y0)/(xc-x0))*((t2)-x0)+y0;
if ycm1(t)<=(((t)-a).^2+((t)-b).^2==r^2) % to ensure that the trajectory is given on the safe region
    yc2=b-r2;
    ycm1=((yc2-y0)/(xc-x0))*((t2)-x0)+y0;
    m=0
end
hold on,plot(t2,ycm1)
t3=a:01:10;
ycm2=((yf-yc2)/(xf-xc))*((t3)-xf)+yf;
hold on,plot(t3,ycm2,'m')

hold on,plot(xc,yc2,'*m') 

