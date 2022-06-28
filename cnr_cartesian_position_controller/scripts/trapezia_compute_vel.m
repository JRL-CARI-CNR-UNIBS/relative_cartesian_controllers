function [v,a]=trapezia_compute_vel(d,t_max,t1,t2)

a=d/(t1^2+(t2-t1)*t1);
v=a*t1;
