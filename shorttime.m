load hwa.mat
%%fix phone axis
x1=Acceleration.X;
y1=Acceleration.Y;
z1=-Acceleration.Z;
x2=x1(1:600);
y2=y1(1:600);
z2=z1(1:600);

mx1=MagneticField.X;
my1=MagneticField.Y;
mz1=-MagneticField.Z;
mx2=mx1(1:600);
my2=my1(1:600);
mz2=mz1(1:600);

ang1=Orientation.X;
ang2=Orientation.Y;
ang3=Orientation.Z;
angx1=ang1(1:600);
angy1=ang2(1:600);
angz1=ang3(1:600);

t=length(x2);

x3=zeros(length(t));
y3=zeros(length(t));
z3=zeros(length(t));

mx3=zeros(length(t));
my3=zeros(length(t));
mz3=zeros(length(t));

for k=1:t
    phi = angx1(k);  
theta = angy1(k);  
psi = angz1(k); 

Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]; 
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];  
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1]; 
R = Rz * Ry * Rx;
accro=[x2(k); y2(k); z2(k)];
magro=[mx2(k); my2(k); mz2(k)];
accor = R' * accro;
magor = R' * magro;
x3(k)=accor(1);
y3(k)=accor(2);
z3(k)=accor(3);
mx3(k)=magor(1);
my3(k)=magor(2);
mz3(k)=magor(3);
end

%%fix direction of NED
acc=[x3; y3; z3]';
mag=[mx3; my3; mz3]';
q=ecompass(acc,mag);
e = eulerd(q,'ZYX','frame');
eux=e(:,3);
euy=e(:,2);
euz=e(:,1);
for k=1:t
    phi2 = eux(k);  
theta2 = euy(k);  
psi2 = euz(k); 

Rx2 = [1 0 0; 0 cos(phi2) -sin(phi2); 0 sin(phi2) cos(phi2)]; 
Ry2 = [cos(theta2) 0 sin(theta2); 0 1 0; -sin(theta2) 0 cos(theta2)];  
Rz2 = [cos(psi2) -sin(psi2) 0; sin(psi2) cos(psi2) 0; 0 0 1]; 
R2 = Rz2 * Ry2 * Rx2;
accro2=[x3(k); y3(k); z3(k)];
accor2 = R' * accro2;
x4(k)=accor2(1);
y4(k)=accor2(2);
z4(k)=accor2(3);
end

%only imu
xsaved=zeros(t,2);
ysaved=zeros(t,2);
for k=1:t
    ax=x4(k)';
    ay=y4(k)';
    [xh, yh]=kalmanf(ax,ay);
    ysaved(k,:)=[xh yh];
end

figure
plot(ysaved(:,1),ysaved(:,2),'-')
xlabel('position X');
ylabel('position Y');
title('IMU');