/*
 * Archivo:     Lab7.c
 * Dispositivo: PIC16F887
 * Autor:       Fernando Jose Caceros Morales
 * Compilador:  XC8 MPLABX V5.40
 * Programa:    Laboratorio7: Contador con PB, Contador con tmr0, 3 disp decimal
 * Hardware:    
 * Creado:      12 de abril de 2021, 10:19 PM
 * Ultima modificacion: 12 de abril de 2021
 */


#include <xc.h>
#pragma config FOSC=INTRC_NOCLKOUT //Oscilador interno sin salida
#pragma config WDTE=OFF           //Reinicio repetitivo del pic
#pragma config PWRTE=ON           //espera de 72 ms al iniciar el pic
#pragma config MCLRE=OFF          //El pin MCLR se utiliza como entrada/salida
#pragma config CP=OFF             //Sin proteccion de codigo
#pragma config CPD=OFF            //Sin proteccion de datos
    
#pragma config BOREN=OFF //Sin reinicio cuando el input voltage es inferior a 4V
#pragma config IESO=OFF  //Reinicio sin cambio de reloj de interno a externo
#pragma config FCMEN=OFF //Cambio de reloj externo a interno en caso de fallas
#pragma config LVP=ON    //Programacion en low voltage permitida
    
//CONFIGURATION WORD 2
#pragma config WRT=OFF //Proteccion de autoescritura por el programa desactivada
#pragma config BOR4V=BOR40V //Reinicio abajo de 4V 

unsigned char Var_Val;
unsigned char Display_No;
unsigned char Display_Val;
unsigned char Centenas;
unsigned char Decenas;
unsigned char Unidades;


unsigned char Tabla7Seg (unsigned char Numero)
{
    unsigned char Codigo;
    switch (Numero)
    {
        case 0:
            Codigo = 0x3F;
            return (Codigo);
            break;
            
        case 1:
            Codigo = 0x06;
            return (Codigo);
            break;
            
        case 2:
            Codigo = 0x5B;
            return (Codigo);
            break;
            
        case 3:
            Codigo = 0x4F;
            return (Codigo);
            break;
            
        case 4:
            Codigo = 0x66;
            return (Codigo);
            break;
            
        case 5:
            Codigo = 0x6D;
            return (Codigo);
            break;
            
        case 6:
            Codigo = 0x7D;
            return (Codigo);
            break;
            
        case 7:
            Codigo = 0x07;
            return (Codigo);
            break;
            
        case 8:
            Codigo = 0x7F;
            return (Codigo);
            break;
            
        case 9:
            Codigo = 0x6F;
            return (Codigo);
            break;
    }
}


void HexToDec (unsigned char Hex)
{
    Centenas = Hex / 100;
    Hex = Hex % 100;
    Decenas = Hex / 10;
    Unidades = Hex % 10;   
}

void __interrupt() isr(void)
{    // only process timer-triggered interrupts
    
    if(RBIF == 1)  
    {
        if (PORTBbits.RB0 == 0) 
        {
            Var_Val++; 
        }
        if (PORTBbits.RB1 == 0) 
        {
            Var_Val--; 
        } 
    }
    if (T0IF == 1) 
    {
        PORTC = PORTC + 1;
        
        TMR0 = 100;
        INTCONbits.T0IF = 0;
        
        PORTD = 0;
        
        switch (PORTE)
        {
            case 1:
                PORTE = 2;
                PORTD = Tabla7Seg (Decenas);
                break;
            case 2:
                PORTE = 4;
                PORTD = Tabla7Seg (Unidades);
                break;
            case 4:
                PORTE = 1;
                PORTD = Tabla7Seg (Centenas);
                break;
        }
        
    }
        
    //PORTA = 0;        
    //TMR0  =  125;
    INTCONbits.RBIF = 0; //clear this interrupt condition    
}

void main(void) 
{
    //configuraciones
    //configuracion reloj
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 0;
    OSCCONbits.IRCF0 = 0;
    OSCCONbits.SCS   = 1;
    //configuracion in out
    ANSELH = 0;
    ANSEL  = 0;
    TRISB  = 3;
    TRISA  = 0;
    TRISC  = 0;
    TRISD  = 0;
    TRISE  = 0;
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1;
    PORTA  = 0;
    PORTB  = 0;
    PORTC  = 0;
    PORTD  = 0x3F;
    PORTE  = 1;
    //configuracion interrupt on change b
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;
    //configuracion tmr0
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA  = 0;
    OPTION_REGbits.PS2  = 0;
    OPTION_REGbits.PS1  = 1;
    OPTION_REGbits.PS0  = 1;
    //reset tmr0
    TMR0 = 100;
    INTCONbits.T0IF = 0;
    //configuracion interrupciones
    INTCONbits.GIE  = 1;
    INTCONbits.RBIE = 1;
    INTCONbits.T0IE = 1;
    Display_No = 0;
    while (1)
    {
        PORTA = Var_Val;
        HexToDec (Var_Val);
    }  
}

