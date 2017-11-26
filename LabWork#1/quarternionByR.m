
%% Quaternião Unitário
% Devolve o respectivo quaternião unitário da Matriz R

function [ vector, phi, QU ] = quarternionByR(R)

    e4 = (1/2) * sqrt( 1 + R(1,1) + R(2,2) + R(3,3) );
    e1 = ( R(3,2) - R(2,3) ) / ( 4 * e4 );
    e2 = ( R(1,3) - R(3,1) ) / ( 4 * e4 );
    e3 = ( R(2,1) - R(1,2) ) / ( 4 * e4 );

    QU = [ e1 e2 e3 e4 ];

    phi = 2 * acos(e4);
    
    rx = e1 / sin( phi/2 );
    ry = e2 / sin( phi/2 );
    rz = e3 / sin( phi/2 );

    vector = [ rx ry rz ]';

end