clc;clear all;clc;

max_lin_vel=1;
max_lin_acc=2;

max_ang_vel=1;
max_ang_acc=2;

dt=2e-3;

x0=zeros(3,1);
xf=0.1*ones(3,1);
last_vel=zeros(3,1);
t=(0:dt:10)';
x=x0;
for idx=1:length(t)

    err=xf-x;

    if idx==1000
        xf=[2;1;1];
    end

    dist=norm(err);
    versor=err/norm(err);
    norm_last_vel=norm(last_vel);


    % 0.5*max_acc*dec_time^2=dec_dist
    % max_acc*dec_time=max_vel -> dec_time=max_vel/max_acc
    % dec_dist=0.5*max_vel^2/max_dec
    dec_dist=0.5*max_lin_vel^2/max_lin_acc;

    if (dist<=dec_dist)
        dec_time=sqrt(2*dist/max_lin_acc);
        dec_vel=max_lin_acc*dec_time;
        vel_no_sat=dec_vel*versor;

    else
        vel_no_sat=max_lin_vel*versor;
    end

    vel_err=vel_no_sat-last_vel;
    if (norm(vel_err)>max_lin_acc*dt)
        acc=max_lin_acc*(vel_err)/norm(vel_err);
        vel=last_vel+acc*dt;
    else
        acc=vel_err/dt;
        vel=vel_no_sat;
    end

    speed(:,idx)=vel';
    pos(:,idx)=x';

    x=x+vel*dt+0.5*acc*dt^2;
    last_vel=vel;
end

subplot(211)
plot(t,pos)
subplot(212)
plot(t,speed)