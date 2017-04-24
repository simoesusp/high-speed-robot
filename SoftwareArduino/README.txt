Os comandos implementados são:

	auto: navegação autônoma, enviando a cada decisão os dados processados dos sonares
    left: habilita flag left
	right: habilita flag right
	both: habilita flags left e right 
	speed: habilita flag speed e desabilita flag freq
	start: anda na velocidade estipulada pelas variáveis pwm_value_l e pwm_value_r caso não haja obstáculo na região próxima (< minDist)
	stop: para
	sweep: aumenta e diminui a velocidade nos motores habilitados pelas flags left e right
	status: exibe de um jeito complicado flags (left, right, speed, freq), velocidades (pwm_value) e frequências setadas
	freq: habilita flag freq e desabilita speed 
	button: exibe o status do safety button
	dist: imprime dados crus dos sonares sem acionar os motores
	log: navegação autônoma por 50 iterações, enviadas posteriormente (RESPOSTA LENTA AO SAFETY BUTTON)
	test: imprime dados crus(ímpares) e processados(pares)
	quiet: navegação autônoma sem feedback do carrinho
	USS: muda o número de sensores ultrassônicos utilizados para tomar decisões para 5 <=> 3.

Pinos:
    nRF2L01+:
       CE_PIN to Arduino pin 5 // é um define
       CSN_PIN to Arduino pin 6 // é um define
       SCK to Arduino pin 13
       MOSI to Arduino pin 11
       MISO to Arduino pin 12

    motores:
        leftmotor 10  // é um define
        rightmotor 3 // é um define

    safety button:
        ON  9 // é um define
        state 8 // é um define
        OFF 7 // é um define

    USS:
        trigPin 2 // é um define
        echoPin0 8 // é um define
        echoPin1 7 // é um define
        echoPin2 A0  // é um define
        echoPin3 A1  // é um define
        echoPin4 A2 // é um define

Obs.:

    0) A flag select define quem é o robô e quem é o controlador. Antes de reprogramar o carrinho, checá-la!!!

    1) A função initTable() inicializa a tabela verdade (vetor T[]).
    1-b) A função obstacleShape() processa os dados dos USS e retorna a posição do vetor T que deve ser acessada para desviar do obstáculo.
    1-c) A função SpeedControl() aplica nos motores a velocidade designada em T[obstacleShape()].

    2) A função SensorSettings() associa cada sensor ao seu respectivo elemento do vetor USS (i.e. grava na struct quais são os pinos usados)

    3) Os nomes dos comandos são definidos na função setInternalCommands().
    3-b) Existe uma flag (Ncommands) que indica o número de comandos existentes, logo para adicionar comandos é necessário atualizá-la.
    3-c) A função action() é quem de fato implementa os comandos recebidos, coloca-os em ação.

    4) A função statusFeedback() é quem determina a informação a ser repassada ao controlador remoto quando o robô está navegando.

    5) Como a cada mensagem que o controlador envia, recebe-se um acknowledgement packet; só imprimo na serial dados não repetidos.

    6) A função start() foi implementada para fazer testes com o intuito de decidir os melhores valores de PWM para cada motor e a posteriori colocá-los nos defines para serem usados nas funções de navegação autônoma.

Defines:

    Ncommands 16 // number of commands available

    valores de PWM dos motores:
        stop_pwm_l 110 // pwm in which left motor is stopped
        stop_pwm_r 90  // pwm in which right motor is stopped
        fullSpeed_l 170
        fullSpeed_r 170
        avgSpeed_l 140
        avgSpeed_r 150
        lowSpeed_l 120
        lowSpeed_r 100

    estratégias de desvio de obstáculo:
        E_L 0 // turn softly to the left
        E_M 1 // turn to the left
        E_F 2 // turn abruptly to the right
        Frente 3 // straight ahead
        D_L 4 // turn softly to the right
        D_M 5 // turn to the right
        D_F 6 // turn abruptly to the right
        FullSpeed 7 // straight ahead

    zonas de perigo, atenção e distante (em cm):
        minDist 50 // obstacle can't get any closer than minDist (in cm) from the robot
        warningDist 350 // minDist < obstacle distance < warningDist => warning zone!
        outOfRange 400


    tamanho do buffer utilizado nas funções log() e test():
        buffer_size 50

    Deadlines (em ms):
        t_max 100  // a cada t_max o controlador envia o status do safety button
        t_start 500 // deadline for loss of radio connection when running - autonomous() as well as start() -
        time_out 60 // USS reading deadline <=> 6.8m (datasheet fala que só vai até 4m...)

    rádio:
        MaxPayload 24 // Maximum message size
        intCmd_size 9 // tamanho máximo de cada comando (em Bytes)

Flags:

    select = 1; // 0: master(remote control) ; 1: slave(robot) 

    all_sensors = 0; // define se serão utilizados todos USS para tomada de decisão (0 -> 3USS , 1 -> todos USS)

    dyn = 1; // define se a leitura dos sonares é dinâmica ou estática (i.e. se for estática, cada leitura terá a duração designada pelo define time_out; caso seja dinâmica, sai da função myPulseIn() assim que todos os sonares detectarem um obstáculo ou time_out seja alcançado.)

            utilizadas para programar as velocidades dos motores.
    left
    right
    speed
    frequency

