function [pos,vel,acc]=trapezia(t,t1,t2,t_max,a,v,d)

if t<0
    pos=0;
    vel=0;
    acc=0;
elseif t<t1
    pos=0.5*a*t^2;
    vel=a*t;
    acc=a;
elseif t<t2
    s1=0.5*a*t1^2;
    pos=s1+v*(t-t1);
    vel=v;
    acc=0;
elseif t<t_max
    s1=0.5*a*t1^2;
    s2=s1+v*(t2-t1);
    dt=t-t2;
    pos=s2+v*dt-0.5*a*dt^2;
    vel=v-a*dt;
    acc=-a;
else
    pos=d;
    vel=0;
    acc=0;
end