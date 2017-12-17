
function vec_ang = parabola(ang_PA, ang_PB, ang_PC, AclMax, vec_t, delta)
    syms th1 th2
    %vec_t = 10*vec_t;
    ti = vec_t(1);
    tdAB = vec_t(2)-vec_t(1);
    tdBC = vec_t(3)-vec_t(2);
    tf = vec_t(3);
    
    %% theta1
    %primeiro segmento
    Acl(1,:) = sign(ang_PB-ang_PA)*abs(AclMax);                     %Acelaração A
    tA(1,:) = tdAB - sqrt(tdAB^2-(2.*(ang_PB-ang_PA))./Acl(1,:));   %Tempo A
    vec_v(1,:) = (ang_PB-ang_PA)./(tdAB-0.5*tA);                    %Velociade AB
    
    %ultimo segmento
    Acl(3,:) = sign(ang_PB-ang_PC)*abs(AclMax);                     %Acelaração C
    tC(1,:) = tdBC-sqrt(tdBC^2+(2.*(ang_PC-ang_PB))./Acl(3,:));          %Tempo C
    vec_v(2,:) = (ang_PC-ang_PB)./(tdBC-0.5*tC);                    %Velociade BC
    
    %Segmento intermedio
    Acl(2,:) = sign(vec_v(2,:)-vec_v(1,:))*abs(AclMax);             %Acelaração B
    tB(1,:) = (vec_v(2,:)-vec_v(1,:))./Acl(2,:);                         %Tempo B
    
    theta(1,:) = ang_PA + 0.5.*Acl(1,:).*tA(1,:).^2;                            %theta para tA
    theta(2,:) = theta(1,:) + vec_v(1,:).*(tdAB-tA(1,:)-0.5.*tB(1,:));          %theta para tB-
    theta(3,:) = theta(2,:) + vec_v(1,:).*tB(1,:) + 0.5*Acl(2,:).*tB(1,:).^2;   %theta para tB+
    theta(4,:) = theta(3,:) + vec_v(2,:).*(tdBC-tC(1,:)-0.5.*tB(1,:));          %theta para tC
    
    %ja sabemos TA TC TB VAB VBC
    
    k=1;
    for t=ti:delta:tf
        for i=1:2
            if((t >= ti)     &&      (t < tA(1,i)) )              %Parabola
                
                vec_ang(k,i) = ang_PA(i) + 0.5*Acl(1,i)*(t-ti)^2;
                
            elseif( (t >= tA(1,i))      &&     (t < (tdAB - 0.5*tB(1,i))) )    %linear
                
                vec_ang(k,i) = theta(1,i) + vec_v(1,i)*(t-tA(1,i));
                
            elseif( (t >= (tdAB - 0.5*tB(1,i)))    &&     (t < (tdAB + 0.5*tB(1,i))) )   %Parabola
                
                vec_ang(k,i) = theta(2,i) + ...
                               vec_v(1,i)*(t-(tdAB-0.5*tB(1,i))) + ...
                               0.5*Acl(2,i)*(t-(tdAB-0.5*tB(1,i)))^2;
            
            elseif( (t >= (tdAB + 0.5*tB(1,i)))      &&       (t < (tf-tC(1,i))) )  %linear             
                
                vec_ang(k,i) = theta(3,i) + ...
                               vec_v(2,i)*(t-(tdAB+0.5*tB(1,i)));


            elseif((t >= (tf-tC(1,i))) && (t < tf))      %parabola 
                
                vec_ang(k,i) = theta(4,i) + ...
                               vec_v(2,i)*(t-(tf-tC(1,i))) + ...
                               0.5*Acl(3,i)*(t-(tf-tC(1,i)))^2;
            end   
        end
        k = k+1;
    end
end