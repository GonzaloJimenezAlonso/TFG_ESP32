#include "DSP28x_Project.h"

#include "menu.h"
#include "sci.h"
#include "math.h"

#include <stdint.h>

// Prototipado de funciones externas
extern void InitAdc(void);
extern void InitSysCtrl(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void SCIA_init();


// Prototipado de funciones internas
void InitAdcRegs(void);
void Gpio_select(void);
void Setup_ePWM1(void);
void Setup_ePWM2(void);
void MensajesInicio (void);
void ModoConfort(void);
void ResetConfort(void);
void ModoAntirrobo(void);
void ResetAntirrobo(void);
float getSonar(void);
void Delay_ms(unsigned long ms);


// Prototipado de funciones de interrupción
interrupt void cpu_timer0_isr(void);
interrupt void adc_SEQ1_isr(void);
interrupt void scia_rx_isr(void);

#define MAX_DISTANCE 400 // Maximum sensor distance in cm



// Variables globales
int duty_cycle_25 = 0; // Indicador de duty cycle actual
volatile int modo_normal = 2; // Indicador de activación del modo normal
unsigned int Vbin_LM35;
unsigned int Vbin_x;
unsigned int Vbin_y;
unsigned int Vbin_z;
float Vana_LM35;
float Vana_x;
float Vana_y;
float Vana_z;
float x_ant = 0.0, y_ant = 0.0, z_ant = 0.0;
int peopl_det = 0;
float mov_total = 0;
float Temp;
char temp_str[8];
int dat_nuevo = 0; //1 Para dato nuevo / 0 para dato antiguo
char modo = 'x';  // 'c' para confort, 'a' para antirrobo
char dat_ant = '\0';
int robo_msg_enviado = 0;
int alarm_active = 0;
int alarm_toggle_counter = 0;
const int alarm_toggle_interval = 5; // 5 * 100ms = 500ms
volatile unsigned int accel_sample_counter = 0;
const unsigned int ACCEL_SAMPLE_PERIOD = 20; // 20 * 100ms = 2 sec
float distance = 0;
char first_time = 1;



//Mensajes para el terminal
const char * hola_msg = "*_Bienvenido!!\n\r*";
const char * conf1_msg = "*_Que modo quieres activar?\n\r*";
const char * conf2_msg = "*_c) Modo Confort\n\r*";
const char * conf3_msg = "*_a) Modo Anti-Robo\n\r*";
const char * conf4_msg = "*_\n\r*";
const char * normal_on_msg = "*_Modo confort ACTIVADO\n\r"
        "\n\r*";
const char * robo_on_msg = "*_Modo Anti-Robo ACTIVADO\n\r"
        "\n\r*";
const char * robo_det = "*_ROBO DETECTADO!!\n\r"
        "\n\r*";

//Mensajes de control
const char * alarm_on = "*SV99*";
const char * alarm_off = "*SV00*";
const char * led_on = "*LR255G0B0*";
const char * led_off = "*LR0G0B0*";
const char * vent_max = "*VR0G255B0*";
const char * vent_med = "*VR255G255B0*";
const char * vent_off = "*VR0G0B0*";
const char * reset_temp = "*n_*";



int Mov_detect = 0;



//###########################################################################
//                      Código Principal
//###########################################################################
void main(void)
{
    InitSysCtrl();           // Inicialización del reloj del sistema SYSCLKOUT=150MHz,
    // deshabilitación del watchdog y
    // habilitación de los relojes de los periféricos

    DINT;                    // Deshabilitación de todas las interrupciones

    Gpio_select();           // Selección de los GPIOs

    Setup_ePWM1();           // Inicialización de la unidad ePWM1

    Setup_ePWM2();           // Inicialización de la unidad ePWM2

    InitCpuTimers();                            // Inicializacion del Timer0

    ConfigCpuTimer(&CpuTimer0,150,100000);      // Se inicializa Timer0 para perido de 100 ms

    InitPieCtrl();                              // Se deshabilitan todas las interrupciones PIE y la INTM

    InitPieVectTable();                         // Inicializacion de la tabla PIE de direcciones

    InitAdc();                                  // Inicialización básica del ADC

    InitAdcRegs();                              // Inicialización resto de registros del ADC

    SCIA_init();                                // Inicializacion de las interrupcion de TX y RX

    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;       // Se direcciona la rutina de interrupcion del Timer0 en la PIE Table
    PieVectTable.SEQ1INT = &adc_SEQ1_isr;
    PieVectTable.SCIRXINTA = &scia_rx_isr;
    EDIS;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;          // Se habilita la interrupcion a nivel de PIE
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;          // Habilitación de interrupción por línea INT1.6 (ADC, SEQ1)
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;          // SCIA RX



    IER |= M_INT1;  // Timer0 y ADC
    IER |= M_INT9;  // SCI

    EINT;                                       // Se habilita la interrupcion global INTM

    CpuTimer0Regs.TCR.bit.TSS = 0;

    GpioDataRegs.GPACLEAR.bit.GPIO17 = 1; // Apagar LED

    MensajesInicio();

    while(1)
    {
        if(modo_normal==1)
        {
            ResetAntirrobo();
            ModoConfort();
        }

        if (modo_normal == 0)
        {
            ResetConfort();
            ModoAntirrobo();
        }
    }
}

void Gpio_select(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;    // GPIO17 como pin de propósito general
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;     // Configuración como salida: LED
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    // GPIO20 como pin de propósito general
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;     // Configuración como entrada: IR
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     // GPIO0 como EPWM1A (ALTAVOZ)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     // GPIO2 como EPWM2A (MOTOR)
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;    // GPIO28 como SCIRXDA
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;    // GPIO29 como SCITXDA (RX HC-05)
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;     // Set GPIO6 as GPIO (trigPin)
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;      // Set GPIO6 as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;     // Set GPIO7 as GPIO (echoPin)
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;      // Set GPIO7 as input
    EDIS;
}

void InitAdcRegs(void)
{
    AdcRegs.ADCTRL1.all = 0;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 1;         // Sample window = 2*(1/ADCCLK)
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 0;       // Modo dual
    AdcRegs.ADCTRL1.bit.CPS = 1;            // CPS = 2 --> ADCCLK = FCLK/(CPS+1)
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;       // Modo start-stop

    AdcRegs.ADCTRL2.all = 0;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;   // Se habilita interrupción SEQ1INT
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;   // Modo de interrupción cada EOS

    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;      // Modo muestreo secuencial
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 1;       // ADCCLKPS = 3 --> FCLK = HSPCLK / 2 * ADCCLKPS
    // HSPCLK = 75 MHz
    // FCLK = 37.5 MHz
    // ADCCLK = 12.5 MHz

    AdcRegs.ADCMAXCONV.all = 0x0003;        // 4 conversiones por secuencia

    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;    // Canal A0 para el sensor de temperatura (LM35)
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 2;    // Canal A2 para el acelerómetro (Eje X)
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 3;    // Canal A3 para el acelerómetro (Eje Y)
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 4;    // Canal A4 para el acelerómetro (Eje Z)

}

void Setup_ePWM1(void)
{
    // Configuración del módulo Time-Base
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;            // Pre-escalador CLKDIV = 1
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;         // Pre-escalador HSPCLKDIV = 2
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;           // Modo up - down
    EPwm1Regs.TBCTL.bit.PRDLD = 0;             // Shadow activado
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 3;          // Sincronización deshabilitada
    EPwm1Regs.TBPRD = 37500;                   // Periodo para 1KHz de fPWM (150MHz/(2*2*37500))

    // Configuración del módulo Compare-Counter
    EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD; // Inicialmente al 0%
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;        // shadow activado para CMPA
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;        // shadow de CMPA cargado en TBCTR = 0

    // Configuración del módulo Action Qualifier
    EPwm1Regs.AQCTLA.all = 0;
    EPwm1Regs.AQCTLA.bit.CAD = 1;              // EPWM1A a low en CMPA down
    EPwm1Regs.AQCTLA.bit.CAU = 2;              // EPWM1A a high en CMPA up
}

void Setup_ePWM2(void)
{
    // Configuración del módulo Time-Base
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;            // Pre-escalador CLKDIV = 1
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;         // Pre-escalador HSPCLKDIV = 2
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;           // Modo up - down
    EPwm2Regs.TBCTL.bit.PRDLD = 0;             // Shadow activado
    EPwm2Regs.TBCTL.bit.SYNCOSEL = 3;          // Sincronización deshabilitada
    EPwm2Regs.TBPRD = 37500;                   // Periodo para 1KHz de fPWM (150MHz/(2*2*37500))

    // Configuración del módulo Compare-Counter
    EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD;              // Asegura el duty cycle en 0% inicialmente
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;        // Shadow activado para CMPA
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;        // Shadow de CMPA cargado en TBCTR = 0

    // Configuración del módulo Action Qualifier
    EPwm2Regs.AQCTLA.all = 0;                  // Limpia AQCTLA
    EPwm2Regs.AQCTLA.bit.ZRO = 1;              // Forzar salida a bajo cuando el contador esté en 0
    EPwm2Regs.AQCTLA.bit.CAU = 2;              // Forzar salida a alto en comparación ascendente (CMPA)
    EPwm2Regs.AQCTLA.bit.CAD = 1;              // Forzar salida a bajo en comparación descendente (CMPA)
}


void MensajesInicio (void)
{
    SCIA_Transmit(hola_msg, strlen(hola_msg));
    SCIA_Transmit(conf1_msg, strlen(conf1_msg));
    SCIA_Transmit(conf2_msg, strlen(conf2_msg));
    SCIA_Transmit(conf3_msg, strlen(conf3_msg));
    SCIA_Transmit(conf4_msg, strlen(conf4_msg));

    SCIA_Transmit(reset_temp,strlen(reset_temp));
}

void ModoConfort(void){
    // Conversión a valores analógicos
    Vana_LM35 = 3000 * (float)Vbin_LM35 / 4095;  // mV
    Temp = Vana_LM35 / 10;                       // Conversión a temperatura en ºC

    // Convert float temperature to integers for whole and decimal parts
    int Temp_whole = (int)Temp;
    int Temp_dec = (int)((Temp - Temp_whole) * 100); // Two decimal places

    // Build temperature string character by character
    temp_str[0] = '*';
    temp_str[1] = 'n';
    temp_str[2] = (Temp_whole / 10) % 10 + '0';  // Tens
    temp_str[3] = Temp_whole % 10 + '0';         // Ones
    temp_str[4] = '.';                           // Decimal point
    temp_str[5] = (Temp_dec / 10) % 10 + '0';    // First decimal place
    temp_str[6] = Temp_dec % 10 + '0';           // Second decimal place
    temp_str[7] = '*';                          // Newline


    SCIA_Transmit(temp_str,strlen(temp_str));


    // Velocidad de ventilador según temperatura
    if (Temp >= 25 && Temp <= 28)
    {
        EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD * 0.5;   // Duty cycle al 25%
        SCIA_Transmit(vent_med,strlen(vent_med));
    }
    else if (Temp > 28)
    {
        EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD * 0.25;  // Duty cycle al 50%
        SCIA_Transmit(vent_max,strlen(vent_max));
    }
    else
    {
        EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD;      // Apaga el PWM cuando la temperatura es menor a 25°C
        SCIA_Transmit(vent_off,strlen(vent_off));
    }
}

void ResetConfort(void)
{
    EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD; //Apagar motor
}



void ModoAntirrobo(void)
{
    if (first_time == 1) {
        // Convert ADC values to analog values
        Vana_x = 3000.0*(float)Vbin_x/4095.0;
        Vana_y = 3000.0*(float)Vbin_y/4095.0;
        Vana_z = 3000.0*(float)Vbin_z/4095.0;

        // Initialize accelerometer reference values
        x_ant = Vana_x;
        y_ant = Vana_y;
        z_ant = Vana_z;

        // Get initial sonar reading and ignore it
        getSonar(); // First reading might be unstable

        // Reset detection flags
        Mov_detect = 0;
        peopl_det = 0;
        alarm_active = 0;

        // Mark initialization complete
        first_time = 0;

        // Skip the rest of the function this first time
        return;
    }

    EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD; //Apagar motor
    Vana_x = 3000.0*(float)Vbin_x/4095.0;
    Vana_y = 3000.0*(float)Vbin_y/4095.0;
    Vana_z = 3000.0*(float)Vbin_z/4095.0;

    float delta_x = fabs(Vana_x - x_ant);
    float delta_y = fabs(Vana_y - y_ant);
    float delta_z = fabs(Vana_z - z_ant);
    mov_total = delta_x + delta_y + delta_z;

    if (accel_sample_counter >= ACCEL_SAMPLE_PERIOD) {
        x_ant = Vana_x;
        y_ant = Vana_y;
        z_ant = Vana_z;
        accel_sample_counter = 0;  // Reset the counter
    }


    if (mov_total > 100 && Mov_detect == 0) {
        Mov_detect = 1;
    }
    if (mov_total < 120 && Mov_detect == 1) {
        Mov_detect = 0;
        robo_msg_enviado = 0;
    }

    distance = getSonar();

    static int dist_counter = 0;

    if (distance <= 40) { //Alguien a menos de 40cm
        peopl_det = 1;
        SCIA_Transmit(led_on, strlen(led_on));
        dist_counter = 0;
    } else {
        peopl_det = 0;
        dist_counter++;
    }

    if ((peopl_det && Mov_detect == 1)&& !alarm_active) {
        alarm_active = 1;
        if (!robo_msg_enviado) {
            SCIA_Transmit(robo_det, strlen(robo_det));
            robo_msg_enviado = 1;
        }
    }

    if (alarm_active) {
        if (alarm_toggle_counter >= alarm_toggle_interval) {
            if (duty_cycle_25 == 0) {
                EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD * 0.1;
                SCIA_Transmit(alarm_on, strlen(alarm_on));
                GpioDataRegs.GPASET.bit.GPIO17 = 1;
            } else {
                EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD * 0.75;
                SCIA_Transmit(alarm_off, strlen(alarm_off));
                GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;
            }
            duty_cycle_25 = !duty_cycle_25;
            alarm_toggle_counter = 0;
        }
    }

    if (dist_counter>=20)
    {
        alarm_active = 0;
        EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD;
        GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;
        SCIA_Transmit(led_off, strlen(led_off));
        SCIA_Transmit(alarm_off, strlen(alarm_off));
    }
}

void ResetAntirrobo(void)
{
    GpioDataRegs.GPACLEAR.bit.GPIO17 = 1; // Apagar LED
    SCIA_Transmit(alarm_off,strlen(alarm_off));
    SCIA_Transmit(led_off,strlen(led_off));
    EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD; //Apagar altavoz
    Mov_detect = 0;
}

float getSonar(void)
{
    // Enviamos el pulso de trigger
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
    DELAY_US(2);
    GpioDataRegs.GPASET.bit.GPIO6 = 1;
    DELAY_US(10);
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

    // Medimos la duración del pulso de echo
    unsigned long pingTime = 0;
    unsigned long timeout = 0;

    while (GpioDataRegs.GPADAT.bit.GPIO7 == 0) {//Esperamos flanco de subida en echo
        timeout++;
        DELAY_US(2);
        if (timeout > 30000) return MAX_DISTANCE; // Return max distance if timeout
    }
    timeout = 0;
    while (GpioDataRegs.GPADAT.bit.GPIO7 == 1)   // Medimos cuanto tiempo está alto
    {
        pingTime++;
        DELAY_US(2);
        if (pingTime > 30000) break;  // 30ms timeout
    }

    // Convertimos a cm (58 = Velocidad_Sonido / 2 / 10000;)
    float distance = (float)pingTime / 58.0;

    return distance;
}

void Delay_ms(unsigned long ms)
{
    unsigned long i;
    for (i = 0; i < (150000 * ms); i++); // Ajuste del tiempo de espera basado en SYSCLKOUT
}


interrupt void cpu_timer0_isr(void)
{
    AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;       // Start ADC conversion

    alarm_toggle_counter++;
    accel_sample_counter++;

    PieCtrlRegs.PIEACK.all = 0x0001;
}

interrupt void adc_SEQ1_isr(void)
{
    Vbin_LM35 = AdcMirror.ADCRESULT0;     // Almacenar resultado del sensor de temperatura
    Vbin_x = AdcMirror.ADCRESULT1;        // Acelerómetro eje X
    Vbin_y = AdcMirror.ADCRESULT2;        // Acelerómetro eje Y
    Vbin_z = AdcMirror.ADCRESULT3;        // Acelerómetro eje Z

    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;       // Reset SEQ1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;     // Clear INT_SEQ1 para habilitar la siguiente interrupción


    PieCtrlRegs.PIEACK.bit.ACK1 = 1;        // Clear PIEACK para habilitar siguiente interrupción
}

interrupt void scia_rx_isr(void)
{
    if(SciaRegs.SCIRXST.bit.RXERROR) {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
        return;
    }

    SCIA_getchar(&modo);

    if(modo == 'c') {
        modo_normal = 1;
        alarm_active = 0;  // Reseteo alarma cuando cambiamos a modo confort
        if (dat_nuevo) {
            SCIA_Transmit(normal_on_msg, strlen(normal_on_msg));
            dat_nuevo = 0;
        }
    }
    else if (modo == 'a') {
        modo_normal = 0;
        first_time = 1;
        if (dat_nuevo) {
            SCIA_Transmit(robo_on_msg, strlen(robo_on_msg));
            dat_nuevo = 0;
        }
    }
    else {
        alarm_active = 0;
        Mov_detect = 0;
        robo_msg_enviado = 0;
        peopl_det = 0;       // Reseteamos detec. de personas
        accel_sample_counter = 0;
        EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD;
        GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;
        SCIA_Transmit(alarm_off, strlen(alarm_off));
        SCIA_Transmit(led_off, strlen(led_off));
    }

    if (modo != dat_ant) {
        dat_nuevo = 1;
        dat_ant = modo;
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


//===========================================================================
// End of SourceCode.
//===========================================================================
