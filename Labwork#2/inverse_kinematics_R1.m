function PCI = inverse_kinematics_R1(T)
n = [0 0 0];
s = [0 0 0];
a = [0 0 0];
t = [0 0 0];


for i = 1:3
    n(i) = T(i,1); %[nx, ny, nz]
    s(i) = T(i,2); %[sx, sy, sz]
    a(i) = T(i,3); %[ax, ay, az]
    t(i) = T(i,4); %[tx, ty, tz]
end


d1 = t(3) - a(3);
d2 = t(2) - a(2);

theta3 = atan2(-a(1),-a(3));
theta4 = atan2(-a(3)*cos(theta3)-a(1)*sin(theta3), a(2));
theta5 = atan2(-n(2), -s(2));


PCI = [d1 d2 theta3 theta4 theta5]; %parâmetros da cinemática inversa para o robot planar 1

end