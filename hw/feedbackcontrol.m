s=0.5;  % Proportional gain for forward speed
a=1;    % Proportional gain for rotational speed
e=0.01; % Proportional gain to adjust robot's final orientation


X=[0;0;0];       % Initial position of robot (x,y,theta)
G=[-0.1;-0.1;0]; % Goal position of robot
maxspeed=0.0285; % Maximum speed the robot can drive

alpha=Inf; rho=Inf; eta=Inf;
figure(1); hold on; set(gcf,'DoubleBuffer','On');
axis([-0.2 0.2 -0.2 0.2]);

plot(G(1),G(2),'*');

tic,
while(abs(rho)>0.01 && sum(X)<10),
% ERROR 
rho =sqrt(sum((X(1:2)-G(1:2)).^2));
alpha=atan2(G(2)-X(2),G(1)-X(1))-X(3);
eta=G(3)-X(3);

% CONTROL LAW
xdot     = min([s*rho; maxspeed]);
thetadot = min([a*alpha+e*eta; maxspeed/0.04]);

% ODOMETRY/MOTION
X(1)=X(1)+cos(X(3))*xdot*0.1;
X(2)=X(2)+sin(X(3))*xdot*0.1;
X(3)=X(3)+thetadot*0.1;

quiver(X(1),X(2),cos(X(3))*xdot,sin(X(3))*xdot);
drawnow;
end;
toc



0.148 x 0.1