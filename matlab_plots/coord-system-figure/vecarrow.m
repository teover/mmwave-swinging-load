function [] = vecarrow(start_angle,end_angle,center,norm,rad,len,tipWidth,tipLength,stemWidth,circLineWidth)
if(vecnorm(norm)==0)
    norm = [1 0 0];
end
norm = norm/vecnorm(norm);
alpha = atan2(norm(1),norm(2));
if(norm(3)~=0)
    phi = start_angle:(end_angle-start_angle)/1000:end_angle;
    theta = atan(-sin(phi+alpha)*sqrt(1/norm(3)^2-1));
elseif(norm(2)==0)
    phi = pi/2;
    theta = start_angle:(end_angle-start_angle)/1000:end_angle;
elseif(norm(1)==0)
    phi = 0;
    theta = start_angle:(end_angle-start_angle)/1000:end_angle;
else
    phi = -alpha;
    theta = start_angle:(end_angle-start_angle)/1000:end_angle;
end
X = center(1) + rad*cos(theta).*cos(phi);
Y = center(2) + rad*cos(theta).*sin(phi);
Z = center(3) + rad*sin(theta);
plot3(X,Y,Z,'k','LineWidth',circLineWidth);
tgt = [X(end)-X(end-1),Y(end)-Y(end-1),Z(end)-Z(end-1)];
mArrow3([X(end),Y(end),Z(end)],[X(end),Y(end),Z(end)]+len*tgt/vecnorm(tgt),'stemWidth',stemWidth,'tipWidth',tipWidth,'tipLength',tipLength);
return