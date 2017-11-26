clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%                                                                  %%')
disp('%%          [Robótica - 07/11/2017 ~ 26/11/2017] LABWORK#3          %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, nº 2011283029                   %%')
disp('%%                   Paulo Almeida, nº 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')


disp('*************************** Exercício 2 ******************************')
%% Robot 5-DOF (3 de rotação e 2 prismáticos): RRPRP



syms theta1 theta2 d3 theta4 d5

% Comprimentos dos elos:
a    = 50;
bmin = 40 ;
c    = 30;


% Junta Rotacional ou Prismatica:
R = 1; P = 0;
%_________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%_________________________________________________________________________________
PJ_DH = [  theta1      a      0    -pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
           theta2      0      0     pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
<<<<<<< Updated upstream
                0     d3      0    -pi/2        0           P;   % Junta Prismática
=======
                0     d3      0    -pi/2       25           P;   % Junta Prismática
>>>>>>> Stashed changes
%_________________________________________________________________________________
           theta4      0      0     pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
                0     d5      0        0       10           P ]; % Junta Prismática
%_________________________________________________________________________________

% A cinematica directa da base   até ao Gripper: 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti  = simplify(Ti) ;


%% POSIÇÃO HOME:

% % O manipulador encontra-se com a configuração "esticado" completamente na
% % vertical e com as juntas prismáticas na configuração mínima;
% 
% bTf = [1  0  0  0  ;
%        0  1  0  0  ;
%        0  0  1  tzh;
%        0  0  0  1  ;]
% 
% % Cinemática Inversa:
% [ q_home ] = inverse_kinematics_ex2(bTf);   


%% INICIALIZAÇÃO DO ROBOT: CRIAR LINKS


for i = 1 : size(PJ_DH,1)
    
    if PJ_DH(i,6) == R              % Juntas Rotacionais
        
        L(i) = Link('d',eval(PJ_DH(i,2)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)));
    end
    
    if PJ_DH(i,6) == P              % Junta Prismática
        
        L(i) = Link('theta',eval(PJ_DH(i,1)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)),...
                    'qlim', [25 50]);
        
    end

end

robot = SerialLink(L, 'name', 'Robot Planar RRR');


<<<<<<< Updated upstream
%% VARIÁVEIS GLOBAIS 

% Inicialização do vector de juntas na nossa posição "home" a começar no
% no ponto inicial [ bmin, -30, 35] dado no enuciado 

% POSIÇÃO HOME:
bTf = [  1  0  0  bmin;
         0  1  0 -30; 
         0  0  1  35;  % 5cm do tapete + 30 do c
         0  0  0  1  ];

[ th1, th2, d3_ ] = inverse_kinematics_ex2(bTf);

alfa = pi - th2;
th4 = alfa;

q = [ th1 th2 d3_ th4 c ]; % Nota: d3 = 0 c/ offset = bmin

% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 theta2 d3 theta4 d5 ]; %

% Construir jacobiana 2 partir dos parâmetros calculados na cinemática inversa
=======
%% VELOCIDADES

% Inicialização do vector de juntas
q = [ 0 0 0 0 0 ];


% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 theta2 d3 theta4 d5 ];
       
% Construir jacobiana a partir dos parâmetros calculados na cinemática inversa
>>>>>>> Stashed changes
Jac = Jacobian(oTg, Ti, q_aux, PJ_DH(:,6));

% retirar as componentes de velocidade nula [ vz wx wy ]
Jac_ = [ Jac(1:2,:); Jac(6,:) ];
        
% Restrição na velocidade linear em y
Vy = 20; %[cm/seg]
        


%% MENU ("main")

% Variaveis MENU
select = 0;
select2 = 0;
STOP = 7;
STOP2 = 3;

while(select ~= STOP)
    
    select = menu('Seleccione a acção a realizar:', 'Cinemática Directa',...
                                                    'Plot do Robo',...
                                                    'alinea a)',...
                                                    'alinea b)',...
                                                    'alinea c)',...
                                                    'Quit');  
                                                
    % Matriz dos parametros de Denavith-Hartenberg: PJ_DH
    if select == 1  
        disp('______________________________________________________________________')
        disp(' ')
        disp('PJ_DH: Matriz dos parametros de Denavith-Hartenberg:')
        disp('______________________________________________________________________')
        disp(' ')
        PJ_DH_ = SerialLink(L, 'name', 'Robot Planar RRPRP')
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
        disp('oTg: Cinematica Directa c/ variaveis simbolicas:')
        disp('______________________________________________________________________')
        disp(' ')
        disp(oTg)
        disp(' ')
        disp('______________________________________________________________________')
    disp('#######################################################################')   
    end  
    
    %% PLOT DO ROBOT:
    
    if select == 2
        figure('units','normalized','outerposition',[0 0 1 1]);
        % Prespectiva de lado do Robot  
        subplot(1,2,1);
<<<<<<< Updated upstream
        robot.plot(q, 'workspace', [-10 60 -40 40 -10 60], 'reach', ... 
=======
        robot.teach(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ... 
>>>>>>> Stashed changes
                       1, 'scale', 10, 'zoom', 0.25); % 'view', 'top', 'trail', 'b.');
                   
                   
        % Prespectiva de topo do Robot -------------------------------------
        
<<<<<<< Updated upstream
        subplot(1,2,2);
        robot.plot(q, 'workspace', [-10 60 -40 40 -10 60],...
                      'reach', 1,...
                      'scale', 10,...
                      'zoom', 0.25,...
                      'view',...
                      'top'); % 'trail', 'b.');
=======
         subplot(1,2,2);
         robot.plot(q, 'workspace', [-10 90 -10 90 -10 90],...
                       'reach', 1,...
                       'scale', 10,...
                       'zoom', 0.25,...
                       'view',...
                       'top'); % 'trail', 'b.');
>>>>>>> Stashed changes
                   
    disp('#######################################################################') 
    end   

    %% a) Calcule a matriz Jacobiana do manipulador ºJ
    if select == 3
        
       


        
        % Inversa da Jacobiana x Velocidades em XYZ 
        qVelocidades = Jac_ \ [ 0 Vy 0 ]';  
        
        
        disp(' ')
        disp('Expresoes para a velocidade das juntas p/ Vx = 10cm/s:')
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
        disp(qVelocidades)
        disp(' ')
        disp('______________________________________________________________________')
        
        
        disp(' ')
        disp('#######################################################################')
    end % fim da alínea a)
    
    %% b) Admitindo que se controla unicamente as juntas de rotação theta1,theta2 e a junta prismática b, 
    %     calcule as velocidades de	 funcionamento que permitem seguir a peçaa
    
<<<<<<< Updated upstream
    if select == 4        
        disp('______________________________________________________________________')
        disp(' ')
        disp('Expressões para velocidade das juntas c/ Vy = 20cm/s:')
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
        % qVelocidades = Inversa da Jacobiana x Velocidades em XYZ / [ vx vy vz]
        % c/ Vy = 20cm/s e Vx e Vz = 0 que permite seguir a peça ao longo do Eixo Y
        % pois só temos temos velocidade em Y. Resultado -> [ W_theta1 W_theta2 V_d3 ] 
        qVelocidades = inv(Jac_)*[ 0 Vy 0]';  
        
        disp(qVelocidades)
        disp(' ')

=======
    if select == 4
        
        %
        
        
        
        % INSERT CODE HERE
        
        
        
        %        
        
>>>>>>> Stashed changes
    disp('#######################################################################')    
    end % fim da alinea b)
    
    %% c) Calcule as expressões	analíticas das variáveis das juntas	?1,?2 e	b em função	dos	
    %     parâmetros da trajetória pretendida para a garra (x=bmin, y variável).
    
    if select == 5
                
<<<<<<< Updated upstream
        % Definir a Home no ponto inicial [ bmin, -30, 35]
      
        % POSIÇÃO HOME:
        bTf = [  1  0  0  bmin;
                 0  1  0 -30; 
                 0  0  1  35;  % 5cm do tapete + 30 do c
                 0  0  0  1  ];

        [ th1, th2, d3_ ] = inverse_kinematics_ex2(bTf);

        alfa = pi - th2;
        th4 = alfa;

        q = [ th1 th2 d3_ th4 c ] % Nota: d3 = 0 c/ offset = bmin
=======
        %
        
        
        
        % INSERT CODE HERE
        
        


        
        %        
>>>>>>> Stashed changes
        
    disp('#######################################################################')   
    end % fim da alinea c)
    

    %% d) Movimento do manipulador com malha de controlo      [FALTA ADAPTAR]
    
    if select == 6
        
       % sub-menu
       while(select2 ~= STOP2)
           

           % Posição home/inicial do Gripper do Robot
           tx = 0; 
           ty = 0;
           tz = a + b + c;

           
<<<<<<< Updated upstream
           % Posição Final do Gripper
           Xf = bTf(1,4); Yf = 30;
                              
           % Período de Amostragem dos Controladores 
=======
           % Perído de Amostragem
>>>>>>> Stashed changes
           h = 0.1;
           
           select2 = menu('Seleccione a abordagem desejada: ', 'Integrador',...
                                                               'Malha-Fechada',...
                                                               'Back');
           %#######################################################################                                                    'Back'); 
           % 1. Abordagem Integradora
           if select2 == 1
               
               for i=1:1:10
                   % Vector de Translação
                   P = [ tx ty 10 ];
                   
                   % Manter orientação do Gripper na posição home/inicial
                   % Significa manter a Matriz de Rotação da posição home/inicial
                   oTg_actual = [ bTf(1:3,1:3); P';

                                  0    0   0    1  ];

                   
                   % Obter os angulos para segund a posição
                   [ q ] = inverse_kinematics_ex1(oTg_actual);
                   
                   % Jacobiando das velocidades nas Juntas
                   qVelocidades = eval(subs(qVelocidades, q_aux, q));
                   
<<<<<<< HEAD
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str((k/32)*100), '%'])
                   
                   % Atendendo que que theta4 e d5 são constantes:
                   q_out(k,:) = [q_controlo(k,1) q_controlo(k,2) q_controlo(k,3) q(4:5)];
<<<<<<< Updated upstream
                   pos_out(k,:) = bTf(1:3,4,k);
=======
=======
                   % Lei de Controlo: Abordagem Integradora
                   q(i+1) = q(i) + h*qVelocidades;
                   
>>>>>>> parent of f5407c3... Merge pull request #2 from fredvaz/master
>>>>>>> Stashed changes
                   
               end
               
<<<<<<< HEAD
               % PLOT do Robô com velocidades
               plot_robot2(robot, k, V, qVelocidades, q_out, pos_out);
               
=======
               % Teste do Plot
               figure('units','normalized','outerposition',[0 0 1 1]);
               % Prespectiva de lado do Robot
               subplot(1,2,1);
               robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ...
                   1, 'scale', 10, 'zoom', 0.25); % 'view', 'top', 'trail', 'b.');
               % Prespectiva de topo do Robot
               subplot(1,2,2);
               robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ...
                   1, 'scale', 10, 'zoom', 0.25, 'view', 'top'); % 'trail', 'b.');
               %
>>>>>>> parent of f5407c3... Merge pull request #2 from fredvaz/master
           end
           %#######################################################################           
           % 2. Abordagem em malha-fechada
           if select2 == 2
               
<<<<<<< Updated upstream
               % Controlo Propocional kp > 0.65 Instavél 
               kp = 0.75; % 0.01; só com o k*h*qVelocidades e c/ 0.1 c/ kd = 0.01 temos bons results!
               % Controlo Derivativo
               kd = 0.01;
               
               % Posição Inicial!
               tx_desej = bTf(1,4);
               ty_desej = bTf(2,4);
               tz_desej = 5; % bTf(3,4)
=======
               %
               
>>>>>>> Stashed changes
               
               
               % INSERT CODE HERE
               
<<<<<<< Updated upstream
               while(bTf(2,4,k) < Yf) % && bTf(1,4) < Xf)
                   
                   % [ Vx vy Vz ]
                   V(k,:) = [ Vx Vy Vz ];
                   
                   % Inversa do Jacobiano x Velocidades em XYZ
                   qVelocidades_ = inv(Jac_)*V(k,:)';
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux,...
                                     [ q_controlo(k,1:3) q(4:5) ] ));
                   
                   % Proximas Juntas: Controlo Malha fechada
                   q_controlo(k+1,:) = q_controlo(k,:) + 1*h*qVelocidades(:,k)';
                   
                   % tx e ty através da Matriz da Cinemática Directa
                   bTf(:,:,k+1) = eval(subs(oTg, q_aux,...
                                [ q_controlo(k,1:3) q(4:5) ] ));
                   
                   % ------------------------------------------------------
                   % Guardamos a Posição actual
                   tx_actual = bTf(1,4,k);
                   ty_actual = bTf(2,4,k);
                   tz_actual = bTf(3,4,k);
                  
                   % Posição desejada = Posição + Deslocamento IDEAL!
                   tx_desej = tx_desej + dx; % IDEALMENTE deve-se manter nos ~37
                   ty_desej = ty_desej + dy;
                   tz_desej = tz_desej + dz; % IDEALMENTE deve-se manter nos 5
                   
                   dx_erro = tx_desej - tx_actual;
                   dy_erro = ty_desej - ty_actual;
                   dz_erro = tz_desej - tz_actual;
                   
                   Vx = kp*dx_erro + kd*dx_erro/h; % dx_erro/h; %
                   Vy = kp*dy_erro + kd*dy_erro/h; % dy_erro/h; %
                   Vz = kp*dz_erro + kd*dz_erro/h; % dz_erro/h; %
                   
                   % NOTA: Como estamos a dividir o deslocamento a efectuar
                   % pelo Período temos de imeadiato a velocidade a impor
                   % no caso c/ parte derivativa do controlo, esta
                   % corresponde acaba por corresponder a velocidade a impor
                   %
                   % Controlamos a velocidade Vx e Vz na tentativa de
                   % corrigir o erro em X e Z 
                   % ------------------------------------------------------        
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str((k/90)*100), '%'])
                  
                   % Atendendo que que theta4 e d5 são constantes:
                   q_out(k,:) = [q_controlo(k,1) q_controlo(k,2) q_controlo(k,3) q(4:5)];
                   pos_out(k,:) = bTf(1:3,4,k);
                   
                   k = k + 1;
               end
=======
>>>>>>> Stashed changes
               
               
               %

               
           end
           %#######################################################################
           
       end % fim do sub-menu
      
 
    disp('#######################################################################')   
    end % fim da alinea d)
    

end % fim do menu/ fim do exercicio

