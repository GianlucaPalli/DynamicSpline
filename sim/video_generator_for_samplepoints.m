Main

%simu=sim('spline_sim1_R2018');%?
simu=sim('spline_sim');%?
long=length(simu.x_axis);

figure
set(gca,'nextplot','replacechildren'); 

v = VideoWriter('videoforspline.avi');
open(v);

grid on
xlabel('x');
ylabel('y');
zlabel('z');
ylim([-0.1 0.1]);
zlim([-1 0]);
view(144,18);

for t=1 : long
   
   Qt=simu.location(t,1:3*Mld.nControlPoints);

   q1= vec2mat(Qt,Mld.nControlPoints);   
   r1 = Mld.b'*q1(1:3,:)';

   x_temp = r1(:,1)';
   
   y_temp = r1(:,2)';
   z_temp = r1(:,3)';
    
   plot3(x_temp,y_temp,z_temp,'r','linewidth',2); 
   
   frame = getframe(gcf);
   writeVideo(v,frame);

end

writeVideo(v,frame);
close(v);





