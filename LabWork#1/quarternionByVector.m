
%% Quaternião Unitário
% Devolve a Matriz R e o respectivo quaternião

function [ R, QU ] = quarternionByVector(r, phi_)
         
    rx = r(1) / ( sqrt(r(1)^2 + r(2)^2 + r(3)^2) );
    ry = r(2) / ( sqrt(r(1)^2 + r(2)^2 + r(3)^2) );
    rz = r(3) / ( sqrt(r(1)^2 + r(2)^2 + r(3)^2) );
        
    e1 = rx * sin(phi_/2);
    e2 = ry * sin(phi_/2);
    e3 = rz * sin(phi_/2);
    e4 = cos(phi_/2);

    QU = [ e1 e2 e3 e4 ];

    R = [ (1-2*e2^2-2*e3^2) 2*(e1*e2-e3*e4) 2*(e1*e3+e2*e4)
           2*(e1*e2+e3*e4) (1-2*e1^2-2*e3^2) 2*(e2*e3-e1*e4)
           2*(e1*e3-e2*e4) 2*(e2*e3+e1*e4) (1-2*e1^2-2*e2^2) ];

end