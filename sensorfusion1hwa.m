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
magro=[mx1(k); my1(k); mz1(k)];
accor = R' * accro;
magor = R' * magro;
x2(k)=accor(1);
y2(k)=accor(2);
z2(k)=accor(3);
mx2(k)=magor(1);
my2(k)=magor(2);
mz2(k)=magor(3);
end

%%fix direction of NED
acc=[x2; y2; z2]';
mag=[mx2; my2; mz2]';
q=ecompass(acc,mag);
e = eulerd(q,'ZYX','frame');
eux=e(:,3);
euy=e(:,2);
euz=e(:,1);
x3=zeros(length(t));
y3=zeros(length(t));
for k=1:t
    phi2 = eux(k);  
theta2 = euy(k);  
psi2 = euz(k); 

Rx2 = [1 0 0; 0 cos(phi2) -sin(phi2); 0 sin(phi2) cos(phi2)]; 
Ry2 = [cos(theta2) 0 sin(theta2); 0 1 0; -sin(theta2) 0 cos(theta2)];  
Rz2 = [cos(psi2) -sin(psi2) 0; sin(psi2) cos(psi2) 0; 0 0 1]; 
R2 = Rz2 * Ry2 * Rx2;
accro2=[x2(k); y2(k); z2(k)];
accor2 = R' * accro2;
x3(k)=accor2(1);
y3(k)=accor2(2);
z3(k)=accor2(3);
end

%only imu
xsaved=zeros(t,2);
ysaved=zeros(t,2);
for k=1:t
    ax=x3(k)';
    ay=y3(k)';
    [xh, yh]=kalmanf(ax,ay);
    ysaved(k,:)=[xh yh];
end


figure
plot(ysaved(:,1),ysaved(:,2),'-')
xlabel('position X');
ylabel('position Y');
title('IMU');

