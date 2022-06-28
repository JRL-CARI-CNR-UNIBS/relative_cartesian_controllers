function [tmax,t1,t2]=trapezia_min_time(d,v,a)
t_acc=v/a;
s=t_acc*v;
if (d>s)
    t_vel=(d-s)/v;
    tmax=t_acc*2+t_vel;
else
    t_acc=sqrt(d/a);
    tmax=2*t_acc;
    t_vel=0;
end

t1=t_acc;
t2=t_acc+t_vel;