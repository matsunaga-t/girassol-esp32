// ======================= DEFINIÇÃO DOS PINOS  ==============
// ============ NÃO MUDAR SEM MOTIVO ========================
    // Pinos dos LDRs
    #define LEFT_LDR_PIN       36     /* Pino do LDR esquerdo */
    #define RIGHT_LDR_PIN      39     /* Pino do LDR direito */

    #define LEFT_LDR2_PIN      34     /* Pino do LDR esquerdo secundário */
    #define RIGHT_LDR2_PIN     35     /* Pino do LDR direito secundário */

    // Pino do servo
    #define MOTOR_OUT          25     // Pino da saída para o servomotor

    // Pinos de entrada de configuração
    #define INPUT1_PIN         32
    #define INPUT2_PIN         33
    #define INPUT3_PIN         35
    #define HALT_PIN           26

// ======================= CONFIGURAÇÕES ADICIONAIS =================
    // LDR esquerdo
    #define LEFT_LDR           2
    // LDR direito
    #define RIGHT_LDR          4

    #define USE_SECONDARY_LDR  0      // 1 para usar os LDRs secundários
    
        // LDR esquerdo secundário
        #define LEFT_LDR2          1
        // LDR direito secundário
        #define RIGHT_LDR2         3

    // ----------------------- Configurações do wifi ------------------------
    #define USE_WIFI           1      // 1 para enviar dados para o servidor

    const char* ssid     = "pi-dashboard";   // rede criada pelo Raspberry Pi (AP)
    const char* password = "dashboard123";
    const char* serverIP = "http://192.168.4.1:5000/dados";

    // ----------------------- Configurações do condicionamento de sinais ----------------
    #define MOVING_AVERAGE_SAMPLES 5  // (int) Quantidade de amostras para a média móvel

    // ......................... Valores do normalizador de LDR ....................
    // ................ NÃO MUDAR SEM MOTIVO .......................................
    #define LDR1_ALPHA           1 / -0.615932450836737f    // (float) Parâmetro alpha do LDR1
    #define LDR2_ALPHA           1 / -0.570998772975310f    // (float) Parâmetro alpha do LDR2
    #define LDR3_ALPHA           1 / -0.578058151321358f    // (float) Parâmetro alpha do LDR3
    #define LDR4_ALPHA           1 / -0.552624264297261f    // (float) Parâmetro alpha do LDR4

    #define LDR1_BETA            powf((320) / expf(11.3373904862202f), LDR1_ALPHA)  // (float) Parâmetro beta do LDR1
    #define LDR2_BETA            powf((320) / expf(10.9327655047729f), LDR2_ALPHA)  // (float) Parâmetro beta do LDR2
    #define LDR3_BETA            powf((320) / expf(11.2481036463693f), LDR3_ALPHA)  // (float) Parâmetro beta do LDR3
    #define LDR4_BETA            powf((320) / expf(10.6852316542604f), LDR4_ALPHA)  // (float) Parâmetro beta do LDR4

    // ----------------------- Configurações de controle ----------------------
    #define CONTROLL_MOTOR     1      // 1 para controlar o servomotor (enviar saída PWM)
    #define ENABLE_HALT        1      // 1 para permitir a parada do controle

    // ......................... Ajuste do PWM .................................
    #define PWM_FREQUENCY        50         // (int) Frequência do PWM
    #define PWM_RESOLUTION       20         // (int) Resolução do PWM
    #define LOWER_CLAMP          1000       // (int) Limite inferior da saída PWM
    #define UPPER_CLAMP          2000       // (int) Limite superior da saída PWM
    #define PWM_CENTER           1500       // (int) Valor central do PWM 

    // ......................... Ajuste do controlador PID ..............................................
    #define PID_CONTROLL_DELAY   10         // (int) Intervalo de tempo, em ms, entre atualizações no PID

    #define P_GAIN               0.120      // (float) Ganho proporcional do PID
    #define I_GAIN               0//.0975   // (float) Ganho integral do PID
    #define D_GAIN               0//.185    // (float) Ganho diferencial do PID

    #define CHANGE_PID_GAIN        1          // 1 para mudar os ganhos do PID

        // ........................... Ajuste dos parâmetros do controlador PID ............................
        #define GAIN_TYPE              0             // Ganho a ser modificado. 0 para Kp, 1 para Ki e 2 para Kd
        #define GAIN_MULT              0.5f          // Multiplicador do ganho na entrada

    // --------------------------- Configurações do acelerômetro ---------------------------------------
    #define USE_ACCELEROMETER      1           // 1 para usar o acelerômetro
    #define ACCEL_PARALLEL_AXIS    x           // ({x|y|z}{Neg|Pos}) Eixo do acelerômetro // ao plano e perp. ao eixo de rotação
    #define ACCEL_NORMAL_AXIS      z           // ({x|y|z}{Neg|Pos}) Eixo do acelerômetro normal ao plano
    #define MAXIMUM_ANGLE          30          // Ângulo máximo em cada sentido, em graus

    // ............................. Limites do acelerõmetro ................................
    #define X_ACCEL_RANGE            2.0         // Faixa de aceleração do eixo X
    #define X_ACCEL_OFFSET           0.0         // Offset da aceleração do eixo X
            
    // --------------------- Configurações de impressão ----------------------------
    #define PRINT_INPUT_INFO 1    // Imprime informação da entrada dos potenciometros
    #define PRINT_PID_GAINS  1    // Imprime os ganhos do controlador PID
    #define PRINT_SYSTEM_IO  1    // Imprime a entrada e saída do sistema

// ==================== MAIS MACROS PARA AJUDAR =====================================
// ==================== NÃO MUDAR SEM MOTIVO ============================
    #define LDR_PARAMS_HELPER(ldr_number) LDR##ldr_number##_ALPHA, LDR##ldr_number##_BETA
    #define LDR_PARAMS(ldr_number)        LDR_PARAMS_HELPER(ldr_number)

    #define XTABLE_LDR \
        /*      pino      numero físisco    entrada crua    classe ms      classe norm        média      iluminância   ilum total*/ \
        X( LEFT_LDR_PIN,     LEFT_LDR,      leftLDR_raw,    leftLDR_ms,    leftLDR_norm,   leftLDR_mean,  leftLDR_E,      leftE) \
        X(RIGHT_LDR_PIN,    RIGHT_LDR,     rightLDR_raw,   rightLDR_ms,   rightLDR_norm,  rightLDR_mean, rightLDR_E,     rightE) \
        XTABLE_LDR_SECONDARY

    #if USE_SECONDARY_LDR
        #define XTABLE_LDR_SECONDARY \
            X2( LEFT_LDR2_PIN,  LEFT_LDR2,  leftLDR2_raw,  leftLDR2_ms,  leftLDR2_norm,  leftLDR2_mean,  leftLDR2_E,   leftE) \
            X2(RIGHT_LDR2_PIN, RIGHT_LDR2, rightLDR2_raw, rightLDR2_ms, rightLDR2_norm, rightLDR2_mean, rightLDR2_E,  rightE)
    #else
        #define XTABLE_LDR_SECONDARY
    #endif

    // key: X(LDR_pin, LDR_number, LDR_raw, LDR_ms, LDR_norm, LDR_mean, LDR_E, total_E)