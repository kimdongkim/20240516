load hwa2.mat
%%fix phone axis
x1=Acceleration.X;
y1=Acceleration.Y;
z1=-Acceleration.Z;

mx1=MagneticField.X;
my1=MagneticField.Y;
mz1=-MagneticField.Z;

ang1=Orientation.X;
ang2=Orientation.Y;
ang3=Orientation.Z;

timestamp = Acceleration.Timestamp;
t=length(timestamp);

x2=zeros(length(t));
y2=zeros(length(t));
z2=zeros(length(t));

mx2=zeros(length(t));
my2=zeros(length(t));
mz2=zeros(length(t));

t=length(x1);

for k=1:t
    phi = ang1(k);  
theta = ang2(k);  
psi = ang3(k); 

Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]; 
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];  
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1]; 
R = Rz * Ry * Rx;
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
title('IMU');