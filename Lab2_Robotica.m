%% CORRER COM O PIONEER 6  

%Daniel Fortunato (81498), Teodoro Dias (81723), Pedro Santos (84162)

clc
clear all

it = 1;
control_it=1;

port = serial_port_start();
pioneer_init(port);
timer = tic;
%Inicialização dos sonares
while toc(timer) < 4.0
    pioneer_set_controls(port, 0,0);
end
son = pioneer_read_sonars();
[x_pos,y_pos,t] = get_rob_pos(0,0,0);

%Rotina sala
while x_pos<2900
    son = pioneer_read_sonars();
    pioneer_set_controls(port, 200,0);
    
    [x_pos,y_pos,t] = get_rob_pos(0,0,0);
    x(it) = x_pos;
    y(it) = y_pos;
    it = it + 1;
end
x_pos
pioneer_set_controls(port,0,0);



%Rodar 90 graus na sala
while t < 42.5*pi/90 %86 graus 
    [x_pos,y_pos,t] = get_rob_pos(0,0,0);
    pioneer_set_controls(port,0,6)
end

pioneer_set_controls(port,0,0);

%Sair da porta da sala
timer_straight = toc(timer);
while toc(timer) - timer_straight < 2
    pioneer_set_controls(port, 200,0);
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
end

while son(8) < 3000
    gain = 0.01;
    son = pioneer_read_sonars();
    angular_velocity = round(gain*(son(1) - son(8)));
    pioneer_set_controls(port, 200,0);
    [x_pos,y_pos,t] = get_rob_pos(0,0,0);
    x(it) = x_pos;
    y(it) = y_pos;
    it = it + 1;
    
end

pioneer_set_controls(port,0,0);
t_adj = t;


%Corredor 1
y_paragem = 17500; %definir
t_ref = 0;
porta1 = 0;
porta8 = 0;
son_ant1 = son(1);
son_ant8 = son(8);
porta_ant1=0;
porta_ant8=0;
ite = 1;

[x_pos,y_pos,t] = get_rob_pos(0,0,t_adj);


while y_pos < y_paragem
    
    son = pioneer_read_sonars();
    
    %Saturação de sensores para valores muito altos de medição
    if (son(1)>1700 || son(8)>1700)  && y_pos>5000
        gain = 0;
        gain_t = 0;
    else
        gain = 0.001*8; %definir-previous *5
        gain_t = gain*1300; %definir 1500 prev
    end
    
    %há uma depressão então há uma porta do lado esquerdo 
    if (son(1)-son_ant1)>=40 && porta1==0 && y_pos>5000
        porta1=1;
    %deixa de haver porta da parte esquerda quando há uma depressão para fora  
    elseif (son(1)-son_ant1)<=-40 && porta1==1 && son(1)<1200
        porta1=0;
    %há uma depressão do lado direito (porta)
    elseif (son(8)-son_ant8)>20 && porta8==0 && y_pos>5000 && y_pos<14000
        porta8=1;   
    %deixa de haver porta do lado esquerdo    
    elseif (son(8)-son_ant8)<=-20 && porta8==1  && y_pos>5000 && y_pos<14000 && son(8)<1000
        porta8=0;
    end
    
    %ajustamento de controlo na presença de portas 
    
    %se estivermos a passar uma porta desligamos os ganhos da velocidade
    %angular
    if porta1==1 || porta8==1
        gain=0;
        gain_t=0;
    %para o caso do início do corredor quando não temos parede do lado
    %direito 
    elseif (porta1==0 && son(8)>1700 && y_pos<5000) 
        left_pos=son(1);
        right_pos = 1600 - left_pos - 269;
    %caso esteja na parede e não haja porta então lê os sonares
    elseif (porta1==0 && y_pos>16000)
        gain=0;
        gain_t=0;
    else
        left_pos=son(1);
        right_pos=son(8);
    end
    
    %calcular quando há uma porta ou quando deixa dehaver porta 
    
    %se entrar na porta do lado esquerdo dá o valor da odometria onde começa 
    if porta1==1 && porta_ant1==0
        [x_prim1,y_prim1,t]=get_rob_pos(0,0,t_adj);
        p1 = 1;
    %se entrar na porta do lado direito dá o valor da odometria onde começa 
    elseif porta8==1 && porta_ant8==0
        [x_prim8,y_prim8,t]=get_rob_pos(0,0,t_adj);
        p8 = 1;
    %enquanto estiver na porta do lado esquerdo mede a distância até à porta
    elseif porta1==1 && porta_ant1==1
        [x1,y1,t1]=get_rob_pos(x_prim1,y_prim1,0);
        if y1>1200
            porta1=0;
        end
        r1 = sqrt(x1^2+y1^2);
        r_vec1(p1) = r1;
        d_vec1(p1) = son(1);
        p1 = p1+1;
    %enquanto estiver na porta do lado direito mede a distância até à porta
    elseif porta8==1 && porta_ant8==1
        [x8,y8,t8]=get_rob_pos(x_prim8,y_prim8,0);
        if y8>1200
            porta8=0;
        end
        r8 = sqrt(x8^2+y8^2);
        r_vec8(p8) = r8;
        d_vec8(p8) = son(8);
        p8 = p8+1;
    %quando sai da porta faz um fit dos pontos das medições que foram
    %feitas do lado esquerdo
    elseif porta1==0 && porta_ant1==1
        coefs1 = polyfit(r_vec1,d_vec1,1);
        rads1 = atan(coefs1(1));
        degreesc1 = rads1*180/pi;
        median(d_vec1)
        %se o algulo da reta dada pelo fit dos pontos for menor que 15
        %então a porta está fechada
        if abs(degreesc1) < 20 && mean(d_vec1)<1500
            filename = 'fechada_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        %se a média das medições feitas for maior que 3000 então há uma
        %porta aberta
        elseif median(d_vec1)>3000
            filename = 'aberta_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs); 
        %caso contrário é uma porta entre-aberta
        else
            filename = 'entreaberta_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs); 
        end
    %quando sai da porta faz um fit dos pontos das medições que foram
    %feitas do lado direito
    elseif porta8==0 && porta_ant8==1
        coefs8 = polyfit(r_vec8,d_vec8,1);
        rads8 = atan(coefs8(1));
        degreesc8 = rads8*180/pi;
        median(d_vec8)
        %se o algulo da reta dada pelo fit dos pontos for menor que 15
        %então a porta está fechada
        if abs(degreesc8) < 20 && mean(d_vec8)<2000
            filename = 'fechada_direita.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        %se a média das medições feitas for maior que 3000 então há uma
        %porta aberta
        elseif median(d_vec8)>3000
             filename = 'aberta_direita.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        %caso contrário é uma porta entre-aberta
        else
            filename = 'entreaberta_direita.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        end
    end
    
    %portas = [porta1 porta8]
    
    %sonares_fixe = [son(1) son(8)]
    
    
    [x_pos,y_pos,t] = get_rob_pos(0,0,t_adj);
     %Compensação de angulo interno
     delta_t = t_ref - t;
    
    %Velocidade/Navegação do robot
    angular_velocity = round(gain*((left_pos) - right_pos)+gain_t*delta_t);
    pioneer_set_controls(port,200,angular_velocity);
    
    
    porta_ant1=porta1;
    porta_ant8=porta8;
    
    %Mais parametros
    son_ant1 = son(1);
    son_ant8 = son(8);
    left(it) = left_pos;
    right(it) = right_pos;
    theta(it) = gain_t*delta_t;
    v_ang(it) = angular_velocity;
    portal(it)=porta1;
    portar(it)=porta8;
    
    
    control1(control_it)=angular_velocity;
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
    control_it=control_it+1;
end



%Rodar 90 graus
t_adj = t_adj + t;
while abs(t) < abs(39.5*pi/90)
    [x_pos,y_pos,t] = get_rob_pos(0,0,t_adj);
    pioneer_set_controls(port,0,-6)
end
t_adj = t_adj + t;
x_adj = x_pos;
y_adj = y_pos;

tim=toc(timer);
while toc(timer)-tim<3.0
    pioneer_set_controls(port, 0,0);
    son=pioneer_read_sonars();
end

if son(1) < 700
    filename = 'fechada_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs);
elseif son(1)>2600
     filename = 'aberta_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs);
else
    filename = 'entreaberta_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs);
end
    


tim = toc(timer);
while toc(timer)-tim < 4.0
%     gain = 0.001*7; %definir-previous *5
%     gain_t = gain*1300; %definir 1500 prev
%     left_pos=son(1);
%     right_pos = 1452 - left_pos - 269;
%     angular_velocity = round(gain*((left_pos+400) - right_pos)+gain_t*delta_t);
    pioneer_set_controls(port,200,0);
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
end

pioneer_set_controls(port,0,0);


%Corredor 2
t_ref = 0;
porta1 = 0;
porta8 = 0;
son_ant1 = son(1);
son_ant8 = son(8);
delta_son8 = 0;
porta_ant1=0;
porta_ant8=0;
ite = 1;
control_it=1;

[x_pos,y_pos,t] = get_rob_pos(x_adj,0,t_adj);

while x_pos < 13000 && (x_pos<11000 || (son(4)>700 && son(5)>700)) %VERIFICAAAAAAR
    
    son = pioneer_read_sonars();
    [x_pos,y_pos,t] = get_rob_pos(x_adj,0,t_adj);
    
    %Saturação de sensores para valores muito altos de medição
    if son(1)>1000 
        gain = 0;
        gain_t = 0;
        [x_pos,y_pos,t] = get_rob_pos(x_adj,0,t_adj);
        x_pos
    else
        gain = 0.001*8; %definir-previous *5
        gain_t = gain*1300; %definir 1500 prev
    end
    
    %há uma depressão então há uma porta do lado esquerdo 
    if (son(1)-son_ant1)>30 && porta1==0 %&& x_pos<11000
        porta1=1;
    %deixa de haver porta da parte esquerda quando há uma depressão para fora  
    elseif (son(1)-son_ant1)<=-30 && porta1==1 && son(1)<800
        porta1=0;
    end
    
    
    %ajustamento de controlo na presença de portas 
    
    %se estivermos a passar uma porta desligamos os ganhos da velocidade
    %angular
    if porta1==1
        gain=0;
        gain_t=0;
    %para o caso do fim do corredor quando não temos parede do lado
    %direito 
    elseif (porta1==0 && son(8)>1700 && y_pos>11000)
        gain=0;
        gain_t=0;
%         left_pos=son(1);
%         right_pos = 1452 - left_pos - 269;
    %caso esteja na parede e não haja porta então lê os sonares
    else
        left_pos=son(1);
        right_pos=son(8);
    end
    
    %se entrar na porta do lado esquerdo dá o valor da odometria onde começa 
    if porta1==1 && porta_ant1==0
        [x_prim1,y_prim1,t]=get_rob_pos(0,0,t_adj);
        p1 = 1;
    %enquanto estiver na porta do lado esquerdo mede a distância até à porta
    elseif porta1==1 && porta_ant1==1
        [x1,y1,t1]=get_rob_pos(x_prim1,y_prim1,0);
        if x1>1200
            porta1=0;
        end
        r1 = sqrt(x1^2+y1^2);
        r_vec1(p1) = r1;
        d_vec1(p1) = son(1);
        p1 = p1+1;
    %quando sai da porta faz um fit dos pontos das medições que foram
    %feitas do lado esquerdo
    elseif porta1==0 && porta_ant1==1
        coefs1 = polyfit(r_vec1,d_vec1,1);
        rads1 = atan(coefs1(1));
        degreesc1 = rads1*180/pi;
        median(d_vec1)
        %se o algulo da reta dada pelo fit dos pontos for menor que 15
        %então a porta está fechada
        if abs(degreesc1) < 30 && mean(d_vec1)<1000
            filename = 'fechada_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        %se a média das medições feitas for maior que 3000 então há uma
        %porta aberta
        elseif median(d_vec1)>2000
            filename = 'aberta_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs); 
        %caso contrário é uma porta entre-aberta
        else
            filename = 'entreaberta_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs); 
        end
    end
    
    
    %Bancos
    if (son(8) - son_ant8 < -50) && porta8 == 0 && son(8) < 1700 && son_ant8 < 1700
        delta_son8 = son(8) - son_ant8;
        porta8 = 1;
        
    elseif ((son(8) - son_ant8 > 50) && porta8 == 1) && son(8) < 1700 && son_ant8 < 1700
        porta8 = 0;
        delta_son8 = 0;
    end
    
    right_pos = son(8) - delta_son8;
    
    %portas = porta1
    
    %sonares_fixe = [son(1) son(8)]
    
     %Compensação de angulo interno
     delta_t = t_ref - t;  
    
    %Velocidade/Navegação do robot
    angular_velocity = round(gain*((left_pos+400) - right_pos)+gain_t*delta_t);
    pioneer_set_controls(port,200,angular_velocity);
    
    
    porta_ant1=porta1;
    porta_ant8=porta8;
    
    %Mais parametros
    son_ant1 = son(1);
    son_ant8 = son(8);
    left(it) = left_pos;
    right(it) = right_pos;
    theta(it) = gain_t*delta_t;
    v_ang(it) = angular_velocity;
    portal(it)=porta1;
    portar(it)=porta8;
    ite = it+1;
    
    control2(control_it)=angular_velocity;
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
    control_it=control_it+1;
    
    
end

porta1=0;

coefs1 = polyfit(r_vec1,d_vec1,1);
rads1 = atan(coefs1(1));
degreesc1 = rads1*180/pi;
median(d_vec1)
%se o algulo da reta dada pelo fit dos pontos for menor que 15
%então a porta está fechada
if abs(degreesc1) < 20 && mean(d_vec1)<2000
    filename = 'fechada_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs);
%se a média das medições feitas for maior que 3000 então há uma
%porta aberta
elseif median(d_vec1)>3000
    filename = 'aberta_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs); 
%caso contrário é uma porta entre-aberta
else
    filename = 'entreaberta_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs); 
end


t_adj = t_adj + t;

%Rodar 90 graus
while abs(t) < abs(39.5*pi/90)
    [x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);
    pioneer_set_controls(port,0,-6)
end



t_adj = t_adj + t;
x_adj = x_adj + x_pos;
y_adj = y_adj + y_pos;

%Andar a direito durante 4 segundos 
tim = toc(timer);
while toc(timer)-tim < 6.0
%     gain = 0.001*7; %definir-previous *5
%     gain_t = gain*1300; %definir 1500 prev
%     left_pos=son(1);
%     right_pos = 1452 - left_pos - 269;
%     angular_velocity = round(gain*((2.1/3)*left_pos - (0.9/3)*right_pos));
    pioneer_set_controls(port, 200,0);
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
end

%Corredor 3 
t_ref = 0;
porta1 = 0;
porta8 = 0;
son_ant1 = son(1);
son_ant8 = son(8);
porta_ant1=0;
porta_ant8=0;
ite = 1;
control_it=1;

[x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);

while (abs(y_pos) < 9000 || son(8)<2000) 
    
    
    son = pioneer_read_sonars();
    
    [x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);
    %Saturação de sensores para valores muito altos de medição
    if son(8)>1700 && son(1)>1700 
        gain = 0;
        gain_t = 0;
        [x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);
        y_pos
    else
        gain = 0.001*10; %definir-previous *5
        gain_t = gain*1000; %definir 1500 prev
    end
    
    %há uma depressão então há uma porta do lado esquerdo 
    if (son(1)-son_ant1)>=30 && porta1==0 
        porta1=1;
    %deixa de haver porta da parte esquerda quando há uma depressão para fora  
    elseif (son(1)-son_ant1)<=-30 && porta1==1 && son(1)<800 %VERIFICAR SE 700 É UM BOM VALOR DE THRESHOLD 
        porta1=0;
    %há uma depressão do lado direito (porta)
    elseif (son(8)-son_ant8)>=20 && porta8==0 &&  abs(y_pos) < 6500
        porta8=1;
        y_fixe = abs(y_pos)
    %deixa de haver porta do lado esquerdo    
    elseif (son(8)-son_ant8)<=-20 && porta8==1 && abs(y_pos) < 6500 && son(8)<1200
        porta8=0;
    end
    
    
    %ajustamento de controlo na presença de portas 
    
    %se estivermos a passar uma porta desligamos os ganhos da velocidade
    %angular
    if porta1==1 || porta8==1 
        gain=0;
        gain_t=0;
    %para o caso do fim do corredor quando não temos parede do lado
    %direito 
    elseif (porta1==0 && son(8)>1700 && abs(y_pos)>10000)
        gain=0;
        gain_t=0;
%         left_pos=son(1);
%         right_pos = 1452 - left_pos - 269;
    %caso esteja na parede e não haja porta então lê os sonares
    else
        left_pos=son(1);
        right_pos=son(8);
    end
    
    
    %calcular quando há uma porta ou quando deixa dehaver porta e verificar
    %se está aberta, fechada ou entre-aberta
    
    %se entrar na porta do lado esquerdo dá o valor da odometria onde começa 
    if porta1==1 && porta_ant1==0
        [x_prim1,y_prim1,t]=get_rob_pos(0,0,t_adj);
        p1 = 1;
    %se entrar na porta do lado direito dá o valor da odometria onde começa 
    elseif porta8==1 && porta_ant8==0
        [x_prim8,y_prim8,t]=get_rob_pos(0,0,t_adj);
        p8 = 1;
    %enquanto estiver na porta do lado esquerdo mede a distância até à porta
    elseif porta1==1 && porta_ant1==1
        [x1,y1,t1]=get_rob_pos(x_prim1,y_prim1,0);
        if abs(y1)>1200
            porta1=0;
        end
        r1 = sqrt(x1^2+y1^2);
        r_vec1(p1) = r1;
        d_vec1(p1) = son(1);
        p1 = p1+1;
    %enquanto estiver na porta do lado direito mede a distância até à porta
    elseif porta8==1 && porta_ant8==1
        [x8,y8,t8]=get_rob_pos(x_prim8,y_prim8,0);
        if abs(y8)>1200
            porta8=0;
        end
        r8 = sqrt(x8^2+y8^2);
        r_vec8(p8) = r8;
        d_vec8(p8) = son(8);
        p8 = p8+1;
    %quando sai da porta faz um fit dos pontos das medições que foram
    %feitas do lado esquerdo
    elseif porta1==0 && porta_ant1==1
        coefs1 = polyfit(r_vec1,d_vec1,1);
        rads1 = atan(coefs1(1));
        degreesc1 = rads1*180/pi;
        median(d_vec1)
        %se o algulo da reta dada pelo fit dos pontos for menor que 15
        %então a porta está fechada
        if abs(degreesc1) < 20 && median(d_vec1)<2000
            filename = 'fechada_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        %se a média das medições feitas for maior que 3000 então há uma
        %porta aberta
        elseif median(d_vec1)>3000
            filename = 'aberta_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs); 
        %caso contrário é uma porta entre-aberta
        else
            filename = 'entreaberta_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs); 
        end
    %quando sai da porta faz um fit dos pontos das medições que foram
    %feitas do lado direito
    elseif porta8==0 && porta_ant8==1
        coefs8 = polyfit(r_vec8,d_vec8,1);
        rads8 = atan(coefs8(1));
        degreesc8 = rads8*180/pi;
        median(d_vec8)
        %se o algulo da reta dada pelo fit dos pontos for menor que 15
        %então a porta está fechada
        if median(d_vec8)<1200 %abs(degreesc8) < 20 &&
            filename = 'fechada_direita.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        %se a média das medições feitas for maior que 3000 então há uma
        %porta aberta
        elseif median(d_vec8)>2000
             filename = 'aberta_direita.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        %caso contrário é uma porta entre-aberta
        else
            filename = 'entreaberta_direita.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        end
    end
    
    %Bancos
    if (son(8) - son_ant8 < -50) && porta8 == 0 && son(8) < 1700 && son_ant8 < 1700 && abs(y_pos)>6000
        delta_son8 = son(8) - son_ant8;
        porta8 = 1;
        
    elseif ((son(8) - son_ant8 > 50) && porta8 == 1) && son(8) < 1700 && son_ant8 < 1700 && abs(y_pos)>6000
        porta8 = 0;
        delta_son8 = 0;
    end
    
    right_pos = son(8) - delta_son8;
    
    
    %portas = [porta1 porta8]
    
    %sonares_fixe = [son(1) son(8)]
    
     %Compensação de angulo interno
     delta_t = t_ref - t;  
    
    %Velocidade/Navegação do robot
    angular_velocity = round(gain*(((2/3)*left_pos) - (1/3)*right_pos)+gain_t*delta_t);
    pioneer_set_controls(port,200,angular_velocity);
    
    
    porta_ant1=porta1;
    porta_ant8=porta8;
    
    %Mais parametros
    son_ant1 = son(1);
    son_ant8 = son(8);
    left(it) = left_pos;
    right(it) = right_pos;
    theta(it) = gain_t*delta_t;
    v_ang(it) = angular_velocity;
    portal(it)=porta1;
    portar(it)=porta8;
    ite = it+1;
    
    control3(control_it)=angular_velocity;
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
    control_it=control_it+1;
    
end

tim = toc(timer);
while toc(timer)-tim < 2.2
    pioneer_set_controls(port,200,0)
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
end

    

%Rodar 90 graus
t_adj = t_adj + t;
while abs(t) < abs(39.7*pi/90)
    [x_pos,y_pos,t] = get_rob_pos(0,0,t_adj);
    pioneer_set_controls(port,0,-6)
end
t_adj = t_adj + t;
x_adj = x_pos;
y_adj = y_pos;

son = pioneer_read_sonars();
son_ant1=son(1);
porta_ant1=1;
porta1=1;
[x_prim1,y_prim1,t]=get_rob_pos(0,0,t_adj);
p1 = 1;



tim = toc(timer);
while toc(timer)-tim < 6.0
    
    son = pioneer_read_sonars();
%     gain = 0.001*7; %definir-previous *5
%     gain_t = gain*1300; %definir 1500 prev
%     left_pos=son(1);
%     right_pos = 1452 - left_pos - 269;
%     angular_velocity = round(gain*((left_pos+400) - right_pos)+gain_t*delta_t);
    if (son(1)-son_ant1)<=-30 && porta1==1 && son(1)<1000 %VERIFICAR SE 700 É UM BOM VALOR DE THRESHOLD 
        porta1=0;
    end
    
    if porta1==1 && porta_ant1==1
        [x1,y1,t1]=get_rob_pos(x_prim1,y_prim1,0);
        if abs(x1)>1200
            porta1=0;
        end
        r1 = sqrt(x1^2+y1^2);
        r_vec1(p1) = r1;
        d_vec1(p1) = son(1);
        p1 = p1+1;
        
    elseif porta1==0 && porta_ant1==1
        coefs1 = polyfit(r_vec1,d_vec1,1);
        rads1 = atan(coefs1(1));
        degreesc1 = rads1*180/pi;
        median(d_vec1)
    %se o algulo da reta dada pelo fit dos pontos for menor que 15
    %então a porta está fechada
        
    end
        
    
    pioneer_set_controls(port,200,0);
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
end

if abs(degreesc1) < 20 && mean(d_vec1)<2000
    filename = 'fechada_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs);
%se a média das medições feitas for maior que 3000 então há uma
%porta aberta
elseif median(d_vec1)>3000
    filename = 'aberta_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs); 
%caso contrário é uma porta entre-aberta
else
    filename = 'entreaberta_esquerda.mp3';
    [s,Fs] = audioread(filename);
    sound(s,Fs); 
end

pioneer_set_controls(port,0,0);


%Corredor 4 
t_ref = 0;
porta1 = 0;
porta8 = 0;
son_ant1 = son(1);
son_ant8 = son(8);
porta_ant1=0;
porta_ant8=0;
ite = 1;
control_it=1;

[x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);

while (son(8)<2000 || abs(x_pos)<8300 )
    
    son = pioneer_read_sonars();
    [x_pos,y_pos,t] = get_rob_pos(x_adj,0,t_adj);
    
    %Saturação de sensores para valores muito altos de medição
    if son(1)>1700 || son(8)>1700
        gain = 0;
        gain_t = 0;
        [x_pos,y_pos,t] = get_rob_pos(x_adj,0,t_adj);
        
    else
        gain = 0.001*5; %definir-previous *5
        gain_t = gain*1300; %definir 1500 prev
    end
    
    %há uma depressão então há uma porta do lado esquerdo 
    if (son(1)-son_ant1)>40 && porta1==0 && abs(x_pos)<8000 && abs(x_pos)>2000
        porta1=1;
    %deixa de haver porta da parte esquerda quando há uma depressão para fora  
    elseif (son(1)-son_ant1)<=-40 && porta1==1 && abs(x_pos)<8000 && son(1)<1200
        porta1=0;
    end
    
    
    %ajustamento de controlo na presença de portas 
    
    %se estivermos a passar uma porta desligamos os ganhos da velocidade
    %angular
    if porta1==1
        gain=0;
        gain_t=0;
    %para o caso do fim do corredor quando não temos parede do lado
    %direito 
    elseif (porta1==0 && son(8)>1700)
        gain=0;
        gain_t=0;
%         left_pos=son(1);
%         right_pos = 1452 - left_pos - 269;
    %caso esteja na parede e não haja porta então lê os sonares
    else
        left_pos=son(1);
        right_pos=son(8);
    end
    
    %se entrar na porta do lado esquerdo dá o valor da odometria onde começa 
    if porta1==1 && porta_ant1==0
        [x_prim1,y_prim1,t]=get_rob_pos(0,0,t_adj);
        p1 = 1;
    %enquanto estiver na porta do lado esquerdo mede a distância até à porta
    elseif porta1==1 && porta_ant1==1
        [x1,y1,t1]=get_rob_pos(x_prim1,y_prim1,0);
        if x1>1200
            porta1=0;
        end
        r1 = sqrt(x1^2+y1^2);
        r_vec1(p1) = r1;
        d_vec1(p1) = son(1);
        p1 = p1+1;
    %quando sai da porta faz um fit dos pontos das medições que foram
    %feitas do lado esquerdo
    elseif porta1==0 && porta_ant1==1
        coefs1 = polyfit(r_vec1,d_vec1,1);
        rads1 = atan(coefs1(1));
        degreesc1 = rads1*180/pi;
        median(d_vec1)
        %se o algulo da reta dada pelo fit dos pontos for menor que 15
        %então a porta está fechada
        if abs(degreesc1) < 20 && mean(d_vec1)<2000
            filename = 'fechada_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs);
        %se a média das medições feitas for maior que 3000 então há uma
        %porta aberta
        elseif median(d_vec1)>3000
            filename = 'aberta_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs); 
        %caso contrário é uma porta entre-aberta
        else
            filename = 'entreaberta_esquerda.mp3';
            [s,Fs] = audioread(filename);
            sound(s,Fs); 
        end
    end
    
    
    %Bancos
    if (son(8) - son_ant8 < -50) && porta8 == 0 && son(8) < 1700 && son_ant8 < 1700
        delta_son8 = son(8) - son_ant8;
        porta8 = 1;
        
    elseif ((son(8) - son_ant8 > 50) && porta8 == 1) && son(8) < 1700 && son_ant8 < 1700
        porta8 = 0;
        delta_son8 = 0;
    end
    
    right_pos = son(8) - delta_son8;
    
    %portas = porta1
    
     %Compensação de angulo interno
     delta_t = t_ref - t;  
    
    %Velocidade/Navegação do robot
    angular_velocity = round(gain*(((1.7/3)*left_pos) - (1.3/3)*right_pos)+gain_t*delta_t);
    pioneer_set_controls(port,200,angular_velocity);
    
    
    porta_ant1=porta1;
    porta_ant8=porta8;
    
    
    
    
    %Mais parametros
    son_ant1 = son(1);
    son_ant8 = son(8);
    left(it) = left_pos;
    right(it) = right_pos;
    theta(it) = gain_t*delta_t;
    v_ang(it) = angular_velocity;
    portal(it)=porta1;
    portar(it)=porta8;
    ite = it+1;
    
    control4(control_it)=angular_velocity;
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
    control_it=control_it+1;
    
    
end

tim = toc(timer);
while toc(timer)-tim < 3.0
    pioneer_set_controls(port,200,0)
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
end

pioneer_set_controls(port,0,0)

t_adj = t_adj + t;
%Rodar 90 graus
while abs(t) < abs(42.5*pi/90)
    [x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);
    pioneer_set_controls(port,0,6)
    
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
end
t_adj = t_adj + t;
x_adj = x_adj + x_pos;
y_adj = y_adj + y_pos;

[x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);

while abs(x_pos) < 3060
    pioneer_set_controls(port,200,0);
    [x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);
    
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
    
end

t_adj = t_adj + t;

while abs(t) < abs(39.5*pi/90)
    [x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);
    pioneer_set_controls(port,0,-6)
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
end

[x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);

while abs(y_pos) < 5000 
    pioneer_set_controls(port,200,0);
    [x_pos,y_pos,t] = get_rob_pos(x_adj,y_adj,t_adj);
    
    [x_print,y_print,t_print] = get_rob_pos(0,0,0);
    
    x(it) = x_print;
    y(it) = y_print;
    it = it + 1;
    
end

pioneer_set_controls(port,0,0);

% figure();
% plot(1:1:it-1,left,'b');
% hold on
% plot(1:1:it-1,right,'r');
% figure();
% plot(1:1:it-1,v_ang,'g');
% hold on
% plot(1:1:it-1,theta,'k');
% figure();
% plot(1:1:it-1,portal,'k');
% hold on
% plot(1:1:it-1,portar,'r');
pioneer_set_controls(port,0,0);
plot_path(x,y)



function plot_path(odo_x,odo_y)
    figure();

    % make data to plot - just a line.
    x_1 = [0.3158,0.3158];
    y_1 = [0.3465,1.644];
    p_1 = [0.3158,1.644];
    
    x_2 = [0.3158,1.694];
    y_2 = [1.644,1.644];
    p_2 = [1.694,1.644];
    
    x_3 = [1.694,1.694];
    y_3 = [1.644,0.3465];
    p_3 = [1.694,0.3465];
    
    x_4 = [1.694,0.3158];
    y_4 = [0.3465,0.3465];
    p_4 = [0.3158,0.3465];

    % NOTE: if your image is RGB, you should use flipdim(img, 1) instead of flipud.

    hold on;
    plot(x_1,y_1,'g--','linewidth',1);
    hold on
    plot(x_2,y_2,'g--','linewidth',1);
    hold on;
    plot(x_3,y_3,'g--','linewidth',1);
    hold on
    plot(x_4,y_4,'g--','linewidth',1);
    hold on
    plot(odo_x/10000,odo_y/10000,'b-*','linewidth',1);
    % set the y-axis back to normal.
    set(gca,'ydir','normal');
end



function [pos_x,pos_y,pos_theta] = get_rob_pos(reg_x,reg_y,reg_t)
    odo = pioneer_read_odometry();
    pos_x = odo(1)-reg_x;
    pos_y = odo(2)-reg_y;
    pos_theta = wrapToPi((odo(3)*360/4096)*pi/180-reg_t);
end
