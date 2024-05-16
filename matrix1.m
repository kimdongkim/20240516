Rover1 = readmatrix('Rover1.xlsx')
%%fix phone axis
x1=Rover1(:,9);
y1=Rover1(:,10);
z1=-Rover1(:,11);

mx1=Rover1(:,15);
my1=Rover1(:,16);
mz1=-Rover1(:,17);

ang1=Rover1(:,6);
ang2=Rover1(:,7);
ang3=Rover1(:,8);


t=length(x1);


for k=1:t
    phi = ang1(k);  
theta = ang2(k);  
psi = ang3(k); 

Rx = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)]; 
Ry = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];  
Rz = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1]; 
R1=inv(Rx);
R2=inv(Ry);
R3=inv(Rz);
R = R1 * R2 * R3;
accro=[x1(k); y1(k); z1(k)];
accor = R' * accro;
x2(k)=accor(1);
y2(k)=accor(2);
z2(k)=accor(3);
end

%only imu
xsaved=zeros(t,2);
ysaved=zeros(t,2);
for k=1:t
    ax=x2(k)';
    ay=y2(k)';
    [xh, yh]=kalmanf(ax,ay);
    ysaved(k,:)=[xh yh];
end

figure
plot(ysaved(:,1),ysaved(:,2),'-')
xlabel('position X');
ylabel('position Y');
title('IMU matrix1');