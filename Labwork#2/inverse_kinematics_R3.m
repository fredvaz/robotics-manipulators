function PCI = inverse_kinematics_R3(T)
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

%%calculos

end