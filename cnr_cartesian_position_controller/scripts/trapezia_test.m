d=.1;
v_max=1;
a_max=5;


[t_max,t1,t2]=trapezia_min_time(d,v_max,a_max);

% c=4;
% t_max=c*t_max;
% t1=c*t1;
% t2=c*t2;

[v,a]=trapezia_compute_vel(d,t_max,t1,t2);

t=linspace(0,t_max,1e3+1)';
for idx=1:length(t)
    [pos(idx,1),vel(idx,1),acc(idx,1)]=trapezia(t(idx),t1,t2,t_max,a,v,d);
end

subplot(3,1,1)
plot(t,pos)
subplot(3,1,2)
plot(t,vel)
subplot(3,1,3)
plot(t,acc)

