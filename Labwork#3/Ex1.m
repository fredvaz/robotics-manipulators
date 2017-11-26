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
disp('*************************** Exercício 1 ******************************')
%% Robô 3-DOF

syms theta1 theta2 theta3

% Cumprimentos dos elos
a2 = 40; d3 = 10;

% Junta Rotacional ou Prismatica
R = 1; P = 0;

%          thetai   di    ai   alfai  offseti  jointtypei
PJ_DH = [  theta1    0    a2       0        0           R;   % Junta Rotacional
           theta2    0    a2       0        0           R;   % Junta Rotacional
           theta3   d3     0       0        0           R ]; % Junta Rotacional + Gripper

% A cinematica directa ate o Gripper 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti = simplify(Ti);


%% INICIALIZAÇÃO DO ROBOT: CRIAR LINKS

% 3 juntas Rotacionais     
for i=1:3
  L(i) = Link('d', eval(PJ_DH(i,2)),...
              'a', eval(PJ_DH(i,3)),...
              'alpha', eval(PJ_DH(i,4)),...
              'offset', eval(PJ_DH(i,5)));
end

robot = SerialLink(L, 'name', 'Robot Planar RRR');


%% VARIÁVEIS GLOBAIS 

% Inicialização do vector de juntas
q = [ 0 0 0 ];

% a) Posição Home
bTf = [ 0  -1   0   50;
        1   0   0   50;
        0   0   1   d3;
        0   0   0   0  ];    
    
% b) Jacobiano 

% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 theta2 theta3 ];
        
% Construir a Jacobiana a partir dos parâmetros calculados na cinemática inversa
Jac = Jacobian(oTg, Ti, q_aux, PJ_DH(:,6));

% Retirar as componentes de velocidade nula [ vz wx wy ]
Jac_ = [ Jac(1:2,:); Jac(6,:) ];
        

%% MENU ("main")
select = 0;
select2 = 0;
STOP = 6;
STOP2 = 3;

while(select ~= STOP)
    
    select = menu('Seleccione a acao a realizar:', 'Cinematica Directa',...
                                                   'Plot do Robo',...
                                                   'alinea a)',...
                                                   'alinea b)',...
                                                   'alinea c)',...
                                                   'sair do programa');    
    % Matriz dos parametros de Denavith-Hartenberg: PJ_DH
    if select == 1  
        disp('______________________________________________________________________')
        disp(' ')
        disp('PJ_DH: Matriz dos parametros de Denavith-Hartenberg:')
        disp('______________________________________________________________________')
        disp(' ')
        PJ_DH_ = SerialLink(L, 'name', 'Robot Planar RRPRP')
        disp('______________________________________________________________________')
        disp(' ')
        disp('oTg: Cinematica Directa c/ variaveis simbolicas:')
        disp('______________________________________________________________________')
        disp(' ')
        disp(oTg)
    disp('#######################################################################')   
    end  
    
    %% PLOT DO ROBOT:
    if select == 2
        figure('units','normalized','outerposition',[0 0 1 1]);
         % Prespectiva de lado do Robô  
        subplot(1,2,1);
        robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ... 
                       1, 'scale', 10, 'zoom', 0.25); % 'view', 'top', 'trail', 'b.');
        % Prespectiva de topo do Robô  
        subplot(1,2,2);
        robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ... 
                       1, 'scale', 10, 'zoom', 0.25, 'view', 'top'); % 'trail', 'b.');
                   
    disp('#######################################################################') 
    end   
    
    %% a) Solucão para as variáveis	das	juntas do robot	q = [ theta1 theta1 theta3 ] que garanta o 
    %     correto posicionamento da ferramenta na posicao "home" do processo de pintura.
    if select == 3
        disp('______________________________________________________________________')
        disp(' ')
        disp('a) Robo na posicao home:')
        disp('______________________________________________________________________')

        % Dada a matriz de Transformação correspondente ao "home" do processo de pintura
        % Obtemos uma solução para as juntas, através da cinemática inversa do robo
        [ q ] = inverse_kinematics_ex1(bTf);

        disp(' ')
        disp(['Segundo ao vector q = [ ' num2str(rad2deg(q(1,1))) 'º ' num2str(rad2deg(q(1,2))) 'º '...
               num2str(rad2deg(q(1,3))) 'º ] extraído através da cinemática inversa.'])

        % Plot do Robo na posição home!
        disp(' ')
        disp('Clique em "Plot do Robô" para o visualizar!')
    
    disp(' ')
    disp('#######################################################################')
    end % fim da alinea a)    
    
    %% b) Jacobiano - Expressões para a velocidade de rotação das juntas 
    %     que asseguram a restrição temporal de pintura (vx=10cm/seg).	
    if select == 4
        disp('______________________________________________________________________')
        disp(' ')
        disp('b) Jacobiano:')
        disp('______________________________________________________________________')
               
        % Restrição na velocidade x em cm/s
        Vx = 10; 
        
        % Inversa da Jacobiana x Velocidades em XYZ 
        qVelocidades = inv(Jac_)*[ Vx 0 0 ]';  
        disp(' ')
        disp('Expressões para a velocidade das juntas c/ Vx = 10cm/s:')
        disp(' ')
        disp(qVelocidades)
     
    disp('#######################################################################')    
    end % fim da alinea b)
    
    %% c) Movimento do manipulador com malha de controlo
    if select == 5
       %% sub-menu
       while(select2 ~= STOP2)
           
           select2 = menu('Seleccione a opcao:', 'Integrador',...
                                                  'Malha-Fechada',...
                                                  'Back'); 
           
           % Restrição imposta no enuciado (e têm que ser negativo
           % para ir no sentido da base)
           Vx = -10;
           % Posição Y segundo a equação da reta y = mx + b
           Vy = Vx;
           
           % Posição Final do Gripper
           Xf = 0; Yf = 0;
                              
           % Período de Amostragem dos Controladores 
           h = 0.1;
           
           % Inicializa as Juntas segundo a Matriz Home/Posição Inicial
           [ q_controlo ] = inverse_kinematics_ex1(bTf);
           
           % 1. Abordagem Integradora (Malha Aberta)
           if select2 == 1
              
               k = 1;
                
               while(bTf(1,4,k) > Xf && bTf(2,4) > Yf)
                
                   % [ Vx vy Vz ]
                   V(k,:) = [ Vx Vy 0 ];
                   
                   % Inversa do Jacobiano x Velocidades em XYZ 
                   qVelocidades_ = inv(Jac_)*V(k,:)';  
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux, q_controlo(k,:)));
                   
                   % Proximas Juntas segundo a Lei de Controlo: Abordagem Integradora
                   q_controlo(k+1,:) = q_controlo(k,:) + h*qVelocidades(:,k)';
                   
                   % tx e ty através da Matriz da Cinemática Directa
                   bTf(:,:,k+1) = eval(subs(oTg, q_aux, q_controlo(k+1,:)));
                   
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str((k/(50*(-Vx*h)))*100), '%']) 
                   % Melhorar isto ^ 
                   k = k + 1;
               end
               
               % PLOT do Robô com velocidades
               plot_robot(robot, k, V, qVelocidades, q_controlo);
               % Temos que melhorar a perfomance dos plots!
              
           end
           
           % 2. Abordagem em malha-fechada
           if select2 == 2

               % Controlo Propocional
               kp = 1.0;
               % Controlo Derivativo
               kd = 0.65;
               
               % Posição Inicial!
               tx_desej = 50;
               ty_desej = 50;
               
               % Deslocamento = Velocidade*Período IDEAL!
               dx = Vx*h;
               dy = Vy*h;
               
               k = 1;
               
               while(bTf(1,4,k) > Xf && bTf(2,4) > Yf)

                   % [ Vx vy Vz ]
                   V(k,:) = [ Vx Vy 0 ];
                 
                   % Inversa do Jacobiano x Velocidades em XYZ 
                   qVelocidades_ = inv(Jac_)*V(k,:)';  
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux, q_controlo(k,:)));
                   
                   % Proximas Juntas: Controlo Malha fechada
                   q_controlo(k+1,:) = q_controlo(k,:) + h*qVelocidades(:,k)';
                   
                   % tx e ty através da Matriz da Cinemática Directa -> f(q(k))
                   bTf(:,:,k+1) = eval(subs(oTg, q_aux, q_controlo(k+1,:)));
                   
                   % ------------------------------------------------------
                   % Guardamos a Posição actual
                   tx_actual = bTf(1,4,k);
                   ty_actual = bTf(2,4,k);
                  
                   % Posição desejada = Posição + Deslocamento IDEAL!
                   tx_desej = tx_desej + dx;
                   ty_desej = ty_desej + dy;
                   
                   dx_erro = tx_desej - tx_actual;
                   dy_erro = ty_desej - ty_actual;
                   
                   Vx = kp*dx_erro + kd*dx_erro/h; 
                   Vy = kp*dy_erro + kd*dy_erro/h;
                   
                   % NOTA: Como estamos a dividir o deslocamento a efectuar
                   % pelo Período temos de imeadiato a velocidade a impor
                   % no caso c/ parte derivativa do controlo, esta
                   % corresponde acaba por corresponder a velocidade a impor
                   % ------------------------------------------------------  
                   
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str((k/(50*(-Vx*h)))*100), '%']) 
                   % Melhorar isto ^ 
                   k = k + 1;
               end
               
               % PLOT do Robô com velocidades
               plot_robot(robot, k, V, qVelocidades, q_controlo);
               % Temos que melhorar a perfomance dos plots!               
             
           end     
           % fim do sub-menu
       end
      
        
    disp('#######################################################################')   
    end % fim da alinea c)
    
end % fim do menu/ fim do exercicio
%disp('#######################################################################')


