;Archivo:		Proyecto1.s
;Dispositivo:		PIC16F887
;Autor;			Fernando Jose Caceros Morales
;Compilador:		pic-as (v2.31) MPLABX V5.40
;
;Programa:		
;Hardware:		Display 7 seg en PORTC con control de mux en PORTD,
;			Leds "semaforo" en PORTA, Push-buttons en PORTB,
;			Leds indicadoras de vía en modificación en PORTE
;
;Creado:		12 marzo, 2021
;Ultima modificacion:	19 marzo, 2021
    
PROCESSOR 16F887
#include <xc.inc>

;configuration word 1
 CONFIG FOSC=INTRC_NOCLKOUT ; Oscilador interno
 CONFIG WDTE=OFF    ; wdt disables (reinicio repetitivo del pic)
 CONFIG PWRTE=ON    ; PWRT enabled (espera de 72ms al iniciar)
 CONFIG MCLRE=OFF   ; El pin de MCLR se utiliza como I/O
 CONFIG CP=OFF	    ; Sin protección de código
 CONFIG CPD=OFF	    ; Sin protección de datos
 
 CONFIG BOREN=OFF   ; Sin reinicio cuándo el voltaje de alimentación baja de 4V
 CONFIG IESO=OFF    ; Reinicio sin cambio de reloj de interno a externo
 CONFIG FCMEN=OFF   ; Cambio de reloj externo a interno en caso de fallo
 CONFIG LVP=ON	    ; programación en bajo voltaje permitida
 
 ;configuration word 2
 CONFIG WRT=OFF	    ; Protección de autoescritura por el programa desactivada
 CONFIG BOR4V=BOR40V ; Reinicio abajo de 4v, (BOR21V=2.1V)
 
 
;*******************************variables a usar********************************
 PSECT udata_bank0 
    Var_Val:		DS 1 ; 1 byte
    Cont_Dec:		DS 1 ; 1 byte
    No_Semaforo:	DS 1 ; 1 byte
    Display_NoSel:	DS 1 ; 1 byte
    Display_Word:	DS 8 ; 8 bytes
    Current_State:	DS 1 ; 1 byte
    TV1:		DS 1 ; 1 byte
    TV1_t:		DS 1 ; 1 byte
    TV2:		DS 1 ; 1 byte
    TV2_t:		DS 1 ; 1 byte
    TV3:		DS 1 ; 1 byte
    TV3_t:		DS 1 ; 1 byte
    Via_1:		DS 1 ; 1 byte
    Via_2:		DS 1 ; 1 byte
    Via_3:		DS 1 ; 1 byte
    Via_Mod:		DS 1 ; 1 byte
    toggle:		DS 1 ; 1 byte
    Delay_1:		DS 1 ; 1 byte
    Delay_2:		DS 1 ; 1 byte
    Delay_3:		DS 1 ; 1 byte
    Delay_4:		DS 1 ; 1 byte
    Nibble_PortA:	DS 2 ; 2 bytes
    Time_Save:		DS 3 ; 3 bytes
    PORTC_t:		DS 1 ; 1 byte
    
PSECT udata_shr ;memoria compartida
    w_temp:		DS 1 ; 1 byte
    status_temp:	DS 1 ; 1 byte
    
;**************************instrucciones vector reset***************************

 PSECT resVector, class=CODE, abs, delta=2
 ORG 00h	    ; posición 0000h para el reset
 resetVector:
    PAGESEL main
    goto main
 
;***************************configuracion del micro*****************************

PSECT intVect, class=CODE,abs, delta=2
ORG 04h
push:
    movwf w_temp
    swapf STATUS, w
    movwf status_temp

isr:
    btfsc RBIF
    call  int_OCB
    
    btfsc T0IF
    call  int_tm0
    
    btfsc TMR1IF
    call  int_tm1
    

pop:
    swapf status_temp, w
    movwf STATUS
    swapf w_temp, f
    swapf w_temp, w
    retfie
 


PSECT code, delta=2, abs
ORG 100h
siete_seg:	    ;tabla de traducción de digito a display
    clrf    PCLATH
    bsf	    PCLATH, 0
    andlw   0Fh
    addwf   PCL, F
    retlw   3Fh	    ; 0
    retlw   06h	    ; 1
    retlw   5Bh	    ; 2
    retlw   4Fh	    ; 3
    retlw   66h	    ; 4
    retlw   6Dh	    ; 5
    retlw   7Dh	    ; 6
    retlw   07h	    ; 7
    retlw   7Fh	    ; 8
    retlw   6Fh	    ; 9
    retlw   77h	    ; A
    retlw   7Ch	    ; B
    retlw   39h	    ; C
    retlw   5Eh	    ; D
    retlw   79h	    ; E
    ;retlw   71h	    ; F
    retlw   40h	    ; -
    
main:
    call    config_IO	    ;inputs PORTB, outputs PORTA, PORTC, PORTD
    call    Tm0_config	    ;Configuración Timer0
    call    Tm1_config	    ;Configuración Timer1
    call    reloj_config    ;Configuración del reloj
    call    config_iocb	    ;Configuración del Interrupt On Change 
    call    config_interrup ;Configuración de las interrupciones
    banksel PORTA	    ;Se selecciona el banco 0
    clrf    PORTA	    ;Se inicializa el puerto A
    clrf    PORTC	    ;Se inicializa el puerto C
    clrf    PORTD	    ;Se inicializa el puerto D
    clrf    PORTE	    ;Se inicializa el puerto E
    movlw   3Fh		    ;Se coloca 0011 1111 en W
    movwf   PORTC	    ;Se mueve W al puerto C
    clrf    Current_State   ;Se inicializa la variable Current_State
    movlw   1		    ;Se coloca 0000 0001 en W
    movwf   Display_NoSel   ;Se mueve W a la variable Display_NoSel
    movwf   Nibble_PortA+0  ;Se mueve W a la variable Nibble_PortA+0
    movwf   Nibble_PortA+1  ;Se mueve W a la variable Nibble_PortA+1
    clrf    Cont_Dec	    ;Se inicializa la variable Cont_Dec
    clrf    Display_Word    ;Se inicializa la variable Display_Word
    clrf    No_Semaforo	    ;Se inicializa la variable No_Semaforo
    movlw   10		    ;Se coloca 0000 1010 en W
    movwf   TV1		    ;Se mueve W a la variable TV1
    movwf   TV1_t	    ;Se mueve W a la variable TV1_t
    movwf   Time_Save+0	    ;Se mueve W a la variable Time_Save+0
    movwf   TV2		    ;Se mueve W a la variable TV2
    movwf   TV2_t	    ;Se mueve W a la variable TV2_t
    movwf   Time_Save+1	    ;Se mueve W a la variable Time_Save+1
    movwf   TV3		    ;Se mueve W a la variable TV3
    movwf   TV3_t	    ;Se mueve W a la variable TV3_t
    movwf   Time_Save+2	    ;Se mueve W a la variable Time_Save+2
    
;********************************loop proncipal*********************************
    
loop:
    call indicador_estado   ;Se llama a la función indicador_estado
    call preparar_displays  ;Se llama a la función preparar_displays
    call Led_Matrix	    ;Se llama a la función Led_Matrix
    goto loop		    ;Regresa al loop		    
    
;**********************************subrutina************************************

config_IO: 
    banksel ANSEL	    ;Se selecciona el banco 3
    
    ;Se colocan los pines como digitales
    clrf    ANSEL	    
    clrf    ANSELH
    
    banksel TRISA	    ;Se selecciona el banco 1
    
    ;PORTB, pines 0, 1 y 2 como entrada
    bsf	    TRISB, 0	    
    bsf	    TRISB, 1
    bsf	    TRISB, 2
    
    ;PORT A, PORTC, PORTD y PORTE como salidas
    clrf    TRISA	    
    clrf    TRISC   
    clrf    TRISD
    clrf    TRISE
    
    bcf	    OPTION_REG, 7   ;Habilita pull-ups
    bsf	    WPUB, 0	    ;Pull-up en pin 0
    bsf	    WPUB, 1	    ;Pull-up en pin 1
    bsf	    WPUB, 2	    ;Pull-up en pin 2
    return

    
    
Tm0_config:
    banksel TRISA	    ;Se selecciona el banco 1
    bcf	    T0CS	    ;Se selecciona el reloj interno
    bcf	    PSA		    ;Se habilita la asignacion de prescaler al Timer0
    
    ;Prescaler 1 a 8
    bcf	    PS2
    bsf	    PS1
    bcf	    PS0	
    
    banksel PORTA	    ;Se selecciona el banco 0
    movlw   150		    ;Se coloca el numero a cargarse hallado con la formula
    movwf   TMR0	    ;Se mueve ese valor al Timer0
    bcf	    T0IF	    ;Se apaga la bandera de interrupcion del Timer0
    return

    
    
Tm1_config:
    banksel PORTA	    ;Se selecciona el banco 1
    bcf	    TMR1CS	    ;Se selecciona el reloj interno
    bsf	    TMR1ON	    ;Se habilita al Timer1
    
    ;Prescaler 1 a 1
    bcf	    T1CKPS1	    
    bcf	    T1CKPS0	     
    
    ;1111 1111  1111 1000 = 65528
    movlw   255		    ;Se coloca el numero a cargarse hallado con la formula
    movwf   TMR1H	    ;Se mueve ese valor al Timer0
    movlw   248		    ;Se coloca el numero a cargarse hallado con la formula
    movwf   TMR1L	    ;Se mueve ese valor al Timer0
    bcf	    TMR1IF	    ;Se apaga la bandera de interrupcion del Timer0
    return

    
    
reloj_config:	
    banksel TRISA	    ;Se selecciona el banco 1
    
    ;Se selecciona una velocidad de reloj a 500KHz
    bcf	    IRCF2	    
    bsf	    IRCF1
    bsf	    IRCF0	     
    return

    
    
config_interrup:
    banksel TRISA	    ;Se selecciona el banco 1
    bsf	    GIE		    ;Se habilita cualquier tipo de interrupción
    bsf	    RBIE	    ;Se habilita la interrupción del puerto B
    bcf	    RBIF	    ;Se apaga la bandera de interrupcion del puerto B
    bsf	    T0IE	    ;Se habilita la interrupción del Timer0
    bcf	    T0IF	    ;Se apaga la bandera de interrupcion del Timer0
    bsf	    TMR1IE	    ;Se habilita la interrupción del Timer1
    bcf	    TMR1IF	    ;Se apaga la bandera de interrupcion del Timer1
    
    return

    
    
config_iocb:
    banksel TRISA	    ;Se selecciona el banco 1
    bsf	    IOCB, 0	    ;Se coloca una interrupcion el pin 0 del puerto B
    bsf	    IOCB, 1	    ;Se coloca una interrupcion el pin 1 del puerto B
    bsf	    IOCB, 2	    ;Se coloca una interrupcion el pin 2 del puerto B
    banksel PORTA	    ;Se selecciona el banco 0
    movf    PORTB, W	    ;Se mueve el valor del puerto B a W
    bcf	    RBIF	    ;Se apaga la bandera de interrupción del puerto B
    return

;***************************rutina de interrupciones****************************

    
int_OCB:
    ;--------------------SELECCION DE ESTADO FSM1-------------------------------
    btfsc   Current_State,0 ;Se revisa el bit 0 de la variable Current_State
    goto    State_1	    ;Se direcciona el código a la sección State_1
    btfsc   Current_State,1 ;Se revisa el bit 1 de la variable Current_State
    goto    State_2	    ;Se direcciona el código a la sección State_2
    btfsc   Current_State,2 ;Se revisa el bit 2 de la variable Current_State
    goto    State_3	    ;Se direcciona el código a la sección State_3
    btfsc   Current_State,3 ;Se revisa el bit 3 de la variable Current_State
    goto    State_4	    ;Se direcciona el código a la sección State_4
    goto    State_0	    ;Se direcciona el código a la sección State_0
    ;---------------------------------------------------------------------------
    State_0:
    btfss   PORTB, 0	    ;Se revisa el pin 0 del puerto B 
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB1
    goto    $+3		    ;Se salta 3 instrucciones si no esta presionado PB1
    
    movlw   1		    ;Se coloca 0000 0001 en W
    movwf   Current_State   ;Se mueve W a la variable del estado actual
    
    bcf	    RBIF	    ;Se apaga la bandera de interrupción
    return
    
    State_1:
    
    btfss   PORTB, 0	    ;Se revisa el pin 0 del puerto B 
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB1
    goto    $+3		    ;Se salta 3 instrucciones si no esta presionado PB1
   
    movlw   2		    ;Se coloca 0000 0010 en W
    movwf   Current_State   ;Se mueve W a la variable del estado actual
    
    ;--------PB2=SUMAR 1, PB3=RESTAR 1, OVERFLOW EN 20, UNDERFLOW EN 10---------
    btfss   PORTB, 1	    ;Se revisa el pin 1 del puerto B 
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB2
    goto    $+11	    ;Se salta 11 instrucciones si no esta presionado PB2
    movlw   20		    ;Se mueve 20 a W
    subwf   TV1_t, F	    ;Se resta 20 al valor TV1_t
    btfsc   CARRY	    ;Si CARRY se apaga, W>F (20>TV1_t) 
    goto    $+5		    ;Se salta las instrucciones de cuando 20>TV1_t
    movlw   20		    ;Se mueve 20 a W
    addwf   TV1_t	    ;Se suma 20 a TV1_t => 20+(TV1_t - 20)= TV1_t
    incf    TV1_t	    ;Se incrementa el valor de TV1_t
    goto    $+3		    ;Se salta las instrucciones de cuando 20 =< TV1_T 
    movlw   10		    ;Se mueve 10 a W
    movwf   TV1_t	    ;Se mueve 10 a TV1_t
    btfss   PORTB, 2	    ;Se revisa el pin 2 del puerto B
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB3
    goto    $+10	    ;Se salta 10 instrucciones si no esta presionado PB3
    movlw   9		    ;Se mueve 9 a W
    decf    TV1_t	    ;Se decrementa el valor de TV1_t
    subwf   TV1_t, F	    ;Se resta 9 al valor TV1_t
    btfss   ZERO	    ;Si ZERO se enciende, TV1_t = 9
    goto    $+4		    ;Se salta las instrucciones de cuando TV1_t != 9
    movlw   20		    ;Se mueve 20 a W
    movwf   TV1_t	    ;Se mueve 20 a TV1_t
    goto    $+2		    ;Se salta a finalizar la interrupción
    addwf   TV1_t	    ;Se suma 9 a TV1_t => 9 +(TV1_t - 9)= TV1_t
    ;---------------------------------------------------------------------------
    
    bcf	    RBIF	    ;Se apaga la bandera de interrupción del puerto B
    return
    
    State_2:
    btfss   PORTB, 0	    ;Se revisa el pin 0 del puerto B 
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB1
    goto    $+3		    ;Se salta 3 instrucciones si no esta presionado PB1
    
    movlw   4		    ;Se coloca 0000 0100 en W
    movwf   Current_State   ;Se mueve W a la variable del estado actual
    
    btfss   PORTB, 1	    ;Se revisa el pin 1 del puerto B 
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB2
    goto    $+11	    ;Se salta 11 instrucciones si no esta presionado PB2
    movlw   20		    ;Se mueve 20 a W
    subwf   TV2_t, F	    ;Se resta 20 al valor TV2_t
    btfsc   CARRY	    ;Si CARRY se apaga, W>F (20>TV2_t)
    goto    $+5		    ;Se salta las instrucciones de cuando 20>TV2_t
    movlw   20		    ;Se mueve 20 a W
    addwf   TV2_t	    ;Se suma 20 a TV2_t => 20+(TV2_t - 20)= TV2_t
    incf    TV2_t	    ;Se incrementa el valor de TV2_t
    goto    $+3		    ;Se salta las instrucciones de cuando 20 =< TV2_T 
    movlw   10		    ;Se mueve 10 a W
    movwf   TV2_t	    ;Se mueve 10 a TV2_t
    btfss   PORTB, 2	    ;Se revisa el pin 2 del puertto B
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB3
    goto    $+10	    ;Se salta 10 instrucciones si no esta presionado PB3
    movlw   9		    ;Se mueve 9 a W
    decf    TV2_t	    ;Se decrementa el valor de TV2_t
    subwf   TV2_t, F	    ;Se resta 9 al valor TV2_t
    btfss   ZERO	    ;Si ZERO se enciende, TV2_t = 9
    goto    $+4		    ;Se salta las instrucciones de cuando TV2_t != 9
    movlw   20		    ;Se mueve 20 a W
    movwf   TV2_t	    ;Se mueve 20 a TV2_t
    goto    $+2		    ;Se salta a finalizar la interrupción
    addwf   TV2_t	    ;Se suma 9 a TV2_t => 9 +(TV2_t - 9)= TV2_t
    
    bcf	    RBIF	    ;Se apaga la bandera de interrupción del puerto B
    return
    
    State_3:
    btfss   PORTB, 0	    ;Se revisa el pin 0 del puerto B 
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB1
    goto    $+3		    ;Se salta 3 instrucciones si no esta presionado PB1
    
    movlw   8		    ;Se coloca 0000 1000 en W
    movwf   Current_State   ;Se mueve W a la variable del estado actual

    btfss   PORTB, 1	    ;Se revisa el pin 1 del puerto B
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB2
    goto    $+11	    ;Se salta 11 instrucciones si no esta presionado PB2
    movlw   20		    ;Se mueve 20 a W
    subwf   TV3_t, F	    ;Se resta 20 al valor TV3_t
    btfsc   CARRY	    ;Si CARRY se apaga, W>F (20>TV3_t)
    goto    $+5		    ;Se salta las instrucciones de cuando 20>TV3_t
    movlw   20		    ;Se mueve 20 a W
    addwf   TV3_t	    ;Se suma 20 a TV3_t => 20+(TV3_t - 20)= TV3_t
    incf    TV3_t	    ;Se incrementa el valor de TV3_t
    goto    $+3		    ;Se salta las instrucciones de cuando 20 =< TV3_T 
    movlw   10		    ;Se mueve 10 a W
    movwf   TV3_t	    ;Se mueve 10 a TV3_t
    btfss   PORTB, 2	    ;Se revisa el pin 2 del puertto B
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB3
    goto    $+10	    ;Se salta 10 instrucciones si no esta presionado PB3
    movlw   9		    ;Se mueve 9 a W
    decf    TV3_t	    ;Se decrementa el valor de TV3_t
    subwf   TV3_t, F	    ;Se resta 9 al valor TV3_t
    btfss   ZERO	    ;Si ZERO se enciende, TV3_t = 9
    goto    $+4		    ;Se salta las instrucciones de cuando TV3_t != 9
    movlw   20		    ;Se mueve 20 a W
    movwf   TV3_t	    ;Se mueve 20 a TV3_t
    goto    $+2		    ;Se salta a finalizar la interrupción
    addwf   TV3_t	    ;Se suma 9 a TV3_t => 9 +(TV3_t - 9)= TV3_t
    
    bcf	    RBIF	    ;Se apaga la bandera de interrupción del puerto B
    return
    
    State_4:
    btfss   PORTB, 0	    ;Se revisa el pin 0 del puerto B 
    goto    $+24	    ;Se salta 24 instrucciones si no esta presionado PB1
    
    ;------------------------------ PB2=ACEPTAR --------------------------------
    btfss   PORTB, 1	    ;Se revisa el pin 1 del puerto B
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB2
    goto    $+12	    ;Se salta 12 instrucciones si no esta presionado PB2
    movf    TV1_t, 0	    ;Se mueve TV1_t a W
    movwf   TV1		    ;Se mueve W a TV1
    movwf   Time_Save	    ;Se mueve W a Time_Save+0
    movf    TV2_t, 0	    ;Se mueve TV2_t a W
    movwf   TV2		    ;Se mueve W a TV2
    movwf   Time_Save+1	    ;Se mueve W a Time_Save+1
    movf    TV3_t, 0	    ;Se mueve TV3_t a W
    movwf   TV3		    ;Se mueve W a TV3
    movwf   Time_Save+2	    ;Se mueve W a Time_Save+2
    ;---------------------------------------------------------------------------
    
    call    Secuencia_Reset ;Se llama a la función Secuencia_Reset
    clrf    PORTA	    ;Se inicializa la variable PORTA
    
    ;------------------------------ PB3=CANCELAR -------------------------------
    btfss   PORTB, 2	    ;Se revisa el pin 2 del puerto B
    goto    $+2		    ;Se salta la instrucción en caso no se presione PB3
    goto    $+9		    ;Se salta 9 instrucciones si no esta presionado PB3
    movf    Time_Save, 0    ;Se mueve Time_Save+0 a W
    movwf   TV1_t	    ;Se mueve W a TV1_t
    movf    Time_Save+1, 0  ;Se mueve Time_Save+1 a W
    movwf   TV2_t	    ;Se mueve W a TV2_t
    movf    Time_Save+2, 0  ;Se mueve Time_Save+2 a W
    movwf   TV3_t	    ;Se mueve W a TV3_t
    ;---------------------------------------------------------------------------
    
    movlw   0		    ;Se coloca 0000 0000 en W
    movwf   Current_State   ;Se mueve W a la variable del estado actual
    
    bcf	    RBIF	    ;Se apaga la bandera de interrupción del puerto B   
    return
   
    
    
    
    
int_tm0:
    banksel PORTA	    ;Se selecciona el banco 0 
    movlw   150		    ;Se coloca el numero a cargarse hallado con la formula
    movwf   TMR0	    ;Se mueve ese valor al Timer0
    bcf	    T0IF	    ;Se apaga la bandera de interrupción del Tiemer0
    
    ;--------------------SELECCION DE DISPLAY A USAR----------------------------
    btfsc   Display_NoSel,0 ;Se revisa si esta encendido el pin 0 en el puerto D
    goto    display_1	    ;Se direcciona a la seccion de interrupcion display_1
    
    btfsc   Display_NoSel,1 ;Se revisa si esta encendido el pin 1 en el puerto D
    goto    display_2	    ;Se direcciona a la seccion de interrupcion display_2
    
    btfsc   Display_NoSel,2 ;Se revisa si esta encendido el pin 2 en el puerto D
    goto    display_3	    ;Se direcciona a la seccion de interrupcion display_3
    
    btfsc   Display_NoSel,3 ;Se revisa si esta encendido el pin 3 en el puerto D
    goto    display_4	    ;Se direcciona a la seccion de interrupcion display_4
    
    btfsc   Display_NoSel,4 ;Se revisa si esta encendido el pin 4 en el puerto D
    goto    display_5	    ;Se direcciona a la seccion de interrupcion display_5
    
    btfsc   Display_NoSel,5 ;Se revisa si esta encendido el pin 5 en el puerto D
    goto    display_6	    ;Se direcciona a la seccion de interrupcion display_6
    
    btfsc   Display_NoSel,6 ;Se revisa si esta encendido el pin 6 en el puerto D
    goto    display_7	    ;Se direcciona a la seccion de interrupcion display_7
    
    btfsc   Display_NoSel,7 ;Se revisa si esta encendido el pin 7 en el puerto D
    goto    display_8	    ;Se direcciona a la seccion de interrupcion display_8
    ;---------------------------------------------------------------------------
    
display_1:
    clrf    PORTD	    ;Se coloca en 0000 0000 el puerto D
    movf    Display_Word, W ;Se coloca el valor de Display_Word+0 en W
    movwf   PORTC_t	    ;Se mueve ese valor al puerto C
    call    Print_PortC
    bsf	    PORTD, 0	    ;Se enciende en el puerto D el pin 0
    goto    next_display    ;Se direcciona a la seccion next_display

display_2:
    clrf    PORTD	    ;Se coloca en 0000 0000 el puerto D
    movf    Display_Word+1,W;Se coloca el valor de Display_Word+1 en W
    movwf   PORTC_t	    ;Se mueve ese valor al puerto C
    call    Print_PortC
    bsf	    PORTD, 1	    ;Se enciende en el puerto D el pin 1
    goto    next_display    ;Se direcciona a la seccion next_display

display_3:
    clrf    PORTD	    ;Se coloca en 0000 0000 el puerto D
    movf    Display_Word+2,W;Se coloca el valor de Display_Word+2 en W
    movwf   PORTC_t	    ;Se mueve ese valor al puerto C
    call    Print_PortC
    bsf	    PORTD, 2	    ;Se enciende en el puerto D el pin 2
    goto    next_display    ;Se direcciona a la seccion next_display

display_4:
    clrf    PORTD	    ;Se coloca en 0000 0000 el puerto D
    movf    Display_Word+3,W;Se coloca el valor de Display_Word+3 en W
    movwf   PORTC_t	    ;Se mueve ese valor al puerto C
    call    Print_PortC
    bsf	    PORTD, 3	    ;Se enciende en el puerto D el pin 3
    goto    next_display    ;Se direcciona a la seccion next_display

display_5:
    clrf    PORTD	    ;Se coloca en 0000 0000 el puerto D
    movf    Display_Word+4,W;Se coloca el valor de Display_Word+4 en W
    movwf   PORTC_t	    ;Se mueve ese valor al puerto C
    call    Print_PortC
    bsf	    PORTD, 4	    ;Se enciende en el puerto D el pin 4
    goto    next_display    ;Se direcciona a la seccion next_display
    
display_6:
    clrf    PORTD	    ;Se coloca en 0000 0000 el puerto D
    movf    Display_Word+5,W;Se coloca el valor de Display_Word+5 en W
    movwf   PORTC_t	    ;Se mueve ese valor al puerto C
    call    Print_PortC
    bsf	    PORTD, 5	    ;Se enciende en el puerto D el pin 5
    goto    next_display    ;Se direcciona a la seccion next_display
    
display_7:
    clrf    PORTD	    ;Se coloca en 0000 0000 el puerto D
    movf    Display_Word+6,W;Se coloca el valor de Display_Word+6 en W
    movwf   PORTC_t	    ;Se mueve ese valor al puerto C
    call    Print_PortC
    bsf	    PORTD, 6	    ;Se enciende en el puerto D el pin 6
    goto    next_display    ;Se direcciona a la seccion next_display
    
display_8:
    clrf    PORTD	    ;Se coloca en 0000 0000 el puerto D
    movf    Display_Word+7,W;Se coloca el valor de Display_Word+7 en W
    movwf   PORTC_t	    ;Se mueve ese valor al puerto C
    call    Print_PortC
    bsf	    PORTD, 7	    ;Se enciende en el puerto D el pin 7
    goto    next_display    ;Se direcciona a la seccion next_display

;-------------------------METODO DE CAMBIO DE DISPLAY---------------------------
next_display:
    
    bcf	    CARRY	    ;Se apaga la señal carry
    btfss   Display_NoSel,7 ;Se revisa si el bit 7 de Display_NoSel
    goto    $+4		    ;Se salta 4 instrucciones si no está encendido
    movlw   1		    ;Se coloca 0000 0001 en W
    movwf   Display_NoSel   ;Se mueve W al puerto D temporal
    goto    $+2		    ;Se salta finalizar la interrupción
    rlf	    Display_NoSel,F ;Se coloca la señal carry (0) a la derecha 
			    ;moviendo el pin encendido a la izquierda
    return

	
	
	
	
int_tm1:
    ;Se crea un delay de 1 segundo aproximadamente
    bsf	    toggle,1	    ;Se enciende el bit 0 de la señal toggle
    movf    Delay_1, w	    ;Se meuve la variable Delay_1 a W
    sublw   38		    ;Se le resta 38 a W
    btfss   ZERO	    ;Se revisa la bandera ZERO
    goto    $+2		    ;Se slata la instrucción de cuando ZERO se enciende
    goto    $+3		    ;Se salta a la segunda serie de delay
    incf    Delay_1	    ;Se incrementa Delay_1
    return
    bcf	    toggle,1	    ;Se enciende el bit 0 de la señal toggle
    movf    Delay_3, w	    ;Se meuve la variable Delay_3 a W
    sublw   38		    ;Se le resta 38 a W
    btfss   ZERO	    ;Se revisa la bandera ZERO
    goto    $+2		    ;Se slata la instrucción de cuando ZERO se enciende
    goto    $+3		    ;Se salta a la segunda serie de delay
    incf    Delay_3	    ;Se incrementa Delay_3
    return
    clrf    Delay_1	    ;Se inicializa la variable Delay_1
    clrf    Delay_3	    ;Se inicialiaz la variable Delay_3
    bcf	    TMR1IF	    ;Se apaga la bandera de interrupción
    
    ;--------------------SELECCION SEMÁFORO DANDO VIA---------------------------
    btfsc   Nibble_PortA+1,0;Se revisa si esta encendido el bit 0 en la variable
    goto    Semaforo_1	    ;Se direcciona a la sección Semaforo_1
    btfsc   Nibble_PortA+1,1;Se revisa si esta encendido el bit 1 en la variable
    goto    Semaforo_2	    ;Se direcciona a la sección Semaforo_2
    btfsc   Nibble_PortA+1,2;Se revisa si esta encendido el pin 2 en la variable
    goto    Semaforo_3	    ;Se direcciona a la sección Semaforo_3
    ;---------------------------------------------------------------------------
    
    bsf	    Nibble_PortA+1,0
    return
    
    Semaforo_1:
    ;--------INDICADOR TIPO DE VIA(VERDE CONTINUO/TITILANTE O AMARILLO)---------
    movf    TV1, w	    ;Se mueve el valor de TV1 a W
    sublw   8		    ;Se le resta 8 
    btfsc   CARRY	    ;Se revisa si el resultado da -1
    goto    $+2		    ;Salta a intrucciones cuando el resultado da -1
    goto    $+12	    ;Salta a instrucciones cuando TV1 >= 8
    movf    TV1, w	    ;Se mueve TV1 a W
    sublw   4		    ;Se le resta 4
    btfsc   CARRY	    ;Se revisa si el resultado da -1
    goto    $+2		    ;Salta a intrucciones cuando el resultado da -1
    goto    $+4		    ;Salta a instrucciones cuando 8 > TV1 >= 4
    ;INSTRUCCIONES 4 > TV1	    (AMARILLO)
    clrf    Nibble_PortA+0  ;Se resetea la variable Nibble_PortA+0	    
    bsf	    Nibble_PortA+0,2;Se enceinde el bit 2 de Nibble_PortA+0
    goto    $+3
    ;INSTRUCCIONES 8 > TV1 >= 4	    (VERDE TITILANTE)
    clrf    Nibble_PortA+0  ;Se resetea la variable Nibble_PortA+0
    bsf	    Nibble_PortA+0,1;Se enceinde el bit 1 de Nibble_PortA+0
    goto    $+3
    ;INSTRUCCIONES TV1 >= 8	    (VERDE CONSTANTE)
    clrf    Nibble_PortA+0  ;Se resetea la variable Nibble_PortA+0
    bsf	    Nibble_PortA+0,0;Se enceinde el bit 0 de Nibble_PortA+0
    
    movf    TV1, w	    ;Se mueve TV1 a W
    sublw   0		    ;Se le resta 0 a W
    btfsc   ZERO	    ;Se revisa si el resultado de la operación fue 0
    goto    $+4		    ;Se salta a cuando TV1 es 0
    ;INSTRUCCIONES TV1 > 0
    movlw   1		    ;Se mueve 1 a W
    subwf   TV1		    ;Se resta 1 a TV1
    goto    $+7		    ;Se salta al final de la interrupción
    ;INSTRUCCIONES TV1 = 0
    movlw   2		    ;Se mueve 0000 0010
    movwf   Nibble_PortA+1  ;Se mueve W a Nibble_PortA+1
    clrf    Nibble_PortA+0  ;Se resetea Nibble_PortA+0
    bsf	    Nibble_PortA+0,0;Se enceinde el bit 0 de Nibble_PortA+0
    movf    Time_Save+0,w   ;Se mueve el valor Time_Save a W
    movwf   TV1		    ;Se mueve W a TV1
    return
    ;---------------------------------------------------------------------------
    
    ;______SE REALIZA EL MISMO PROCEDIMIENTO CON LOS TIEMPOS DE VIA 2 Y 3_______
    Semaforo_2:
    movf    TV2, w
    movwf   Via_1
    movwf   Via_2
    movwf   Via_3
    
    movf    TV2, w
    sublw   8
    btfsc   CARRY
    goto    $+2
    goto    $+12
    movf    TV2, w
    sublw   4
    btfsc   CARRY
    goto    $+2
    goto    $+4
    clrf    Nibble_PortA+0
    bsf	    Nibble_PortA+0,2
    goto    $+3
    clrf    Nibble_PortA+0
    bsf	    Nibble_PortA+0,1
    goto    $+3
    clrf    Nibble_PortA+0
    bsf	    Nibble_PortA+0,0
    movf    TV2, w
    sublw   0
    btfsc   ZERO
    goto    $+4
    movlw   1
    subwf   TV2
    goto    $+7
    movlw   4
    movwf   Nibble_PortA+1
    clrf    Nibble_PortA+0
    bsf	    Nibble_PortA+0,0
    movf    Time_Save+1,w
    movwf   TV2
    return

    Semaforo_3:
    movf    TV3, w
    movwf   Via_1
    movwf   Via_2
    movwf   Via_3
    
    movf    TV3, w
    sublw   8
    btfsc   CARRY
    goto    $+2
    goto    $+12
    
    movf    TV3, w
    sublw   4
    btfsc   CARRY
    goto    $+2
    goto    $+4
    clrf    Nibble_PortA+0
    bsf	    Nibble_PortA+0,2
    goto    $+3
    clrf    Nibble_PortA+0
    bsf	    Nibble_PortA+0,1
    goto    $+3
    
    clrf    Nibble_PortA+0
    bsf	    Nibble_PortA+0,0
    
    movf    TV3, w
    sublw   0
    btfsc   ZERO
    goto    $+4
    
    movlw   1
    subwf   TV3
    goto    $+7
    
    movlw   1
    movwf   Nibble_PortA+1
    clrf    Nibble_PortA+0
    bsf	    Nibble_PortA+0,0
    
    movf    Time_Save+2,w
    movwf   TV3
    
    return
	
;*********************************subrutinas************************************
	

preparar_displays:
    ;---------------- ASIGNACIÓN DE VALORES A DISPLAYs V1,V2 Y V3---------------
    movlw   1		    ;Se coloca en W 0000 0001
    movwf   No_Semaforo	    ;Se mueve W a No_Semaforo
    bcf	    CARRY	    ;Se apaga la bandera CARRY
    call    Via_select	    ;Se llama a la función Via_select
    movwf   Var_Val	    ;Se mueve el valor de W a Var_Val
    movlw   10		    ;Se pone en W un valor de 10 
    subwf   Var_Val, F	    ;Se resta 10 a Var_Val 
    incf    Cont_Dec	    ;Se incrementa el contador de decenas
    btfsc   CARRY	    ;Se revisa si la bandera de carry se apaga 
    goto    $-3		    ;Se regresa 3 instrucciones en el código
    decf    Cont_Dec	    ;Se decrementa el contador de centenas
    addwf   Var_Val	    ;Se suma 10 a Var_Val 
    movf    Cont_Dec,w	    ;Se mueve el valor del contador a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción 
    movwf   Display_Word    ;Se mueve W de la tabla a Display_Word+0
    movf    Var_Val, w	    ;Se mueve el valor de Var_Val a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+1  ;Se mueve W de la tabla a Display_Word+1
    clrf    Cont_Dec	    ;Se reseta la variable Cont_Dec
    ;---------------------------------------------------------------------------
    
    movlw   2		    ;Se coloca en W 0000 0010
    movwf   No_Semaforo	    ;Se mueve W a No_Semaforo
    bcf	    CARRY	    ;Se apaga la bandera CARRY
    call    Via_select	    ;Se llama a la función Via_select
    movwf   Var_Val	    ;Se mueve el valor de W a Var_Val
    movlw   10		    ;Se pone en W un valor de 10 
    subwf   Var_Val, F	    ;Se resta 10 a Var_Val 
    incf    Cont_Dec	    ;Se incrementa el contador de decenas
    btfsc   CARRY	    ;Se revisa si la bandera de carry se apaga 
    goto    $-3		    ;Se regresa 3 instrucciones en el código
    decf    Cont_Dec	    ;Se decrementa el contador de centenas
    addwf   Var_Val	    ;Se suma 10 a Var_Val
    movf    Cont_Dec,w	    ;Se mueve el valor del contador a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción 
    movwf   Display_Word+2  ;Se mueve W de la tabla a Display_Word+2
    movf    Var_Val, w	    ;Se mueve el valor de Var_Val a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+3  ;Se mueve W de la tabla a Display_Word+3
    clrf    Cont_Dec	    ;Se reseta la variable Cont_Dec
    
    movlw   4		    ;Se coloca en W 0000 0100
    movwf   No_Semaforo	    ;Se mueve W a No_Semaforo
    bcf	    CARRY	    ;Se apaga la bandera CARRY
    call    Via_select	    ;Se llama a la función Via_select
    movwf   Var_Val	    ;Se mueve el valor de W a Var_Val
    movlw   10		    ;Se pone en W un valor de 10 
    subwf   Var_Val, F	    ;Se resta 10 a Var_Val 
    incf    Cont_Dec	    ;Se incrementa el contador de decenas
    btfsc   CARRY	    ;Se revisa si la bandera de carry se apaga 
    goto    $-3		    ;Se regresa 3 instrucciones en el código
    decf    Cont_Dec	    ;Se decrementa el contador de centenas
    addwf   Var_Val	    ;Se suma 10 a Var_Val
    movf    Cont_Dec,w	    ;Se mueve el valor del contador a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+4  ;Se mueve W de la tabla a Display_Word+4
    movf    Var_Val, w	    ;Se mueve el valor de Var_Val a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+5  ;Se mueve W de la tabla a Display_Word+5
    clrf    Cont_Dec	    ;Se reseta la variable Cont_Dec
    
    ;------------ASIGNACIÓN DE VALORES A DISPLAY VIA EN MODIFICACION------------
    
    bcf	    CARRY	    ;Se apaga la bandera CARRY
    call    Via_Mod_select  ;Se llama a la fucnión Via_Mod_select
    btfsc   Current_State,0 ;Se revisa si Current_State tiene un valor 0000 0001
    goto    $+6		    ;Se salta a instrucciones de asignacion de velores
    btfsc   Current_State,1 ;Se revisa si Current_State tiene un valor 0000 0010
    goto    $+4		    ;Se salta a instrucciones de asignacion de velores
    btfsc   Current_State,2 ;Se revisa si Current_State tiene un valor 0000 0100
    goto    $+2		    ;Se salta a instrucciones de asignacion de velores
    goto    $+18	    ;Se salta a la asignación para los estados 0 y 4
    ;ASIGNACION DE VALORES COMO DISPLAY WORD
    movf    Via_Mod, w	    ;Se mueve el valor de Via_Mod a W
    movwf   Var_Val	    ;Se mueve el valor de W a Var_Val
    movlw   10		    ;Se pone en W un valor de 10
    subwf   Var_Val, F	    ;Se resta 10 a Var_Val 
    incf    Cont_Dec	    ;Se incrementa el contador de decenas
    btfsc   CARRY	    ;Se revisa si la bandera de carry se apaga 
    goto    $-3		    ;Se regresa 3 instrucciones en el código
    decf    Cont_Dec	    ;Se decrementa el contador de centenas
    addwf   Var_Val	    ;Se suma 10 a Var_Val
    movf    Cont_Dec,w	    ;Se mueve el valor del contador a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+6  ;Se mueve W de la tabla a Display_Word+6
    movf    Var_Val, w	    ;Se mueve el valor de Var_Val a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+7  ;Se mueve W de la tabla a Display_Word+7
    clrf    Cont_Dec	    ;Se reseta la variable Cont_Dec
    goto    $+15	    ;Se salta a finalizar la interrupción
    ;ASIGNACION DE VALORES PARA S1 y S4
    btfss   Current_State,3 ;Se revisa si Current_State tiene un valor 0000 1000
    goto    $+2		    ;Se salta a instrucciones para Estado 1
    goto    $+6		    ;Se salta a instrucciones para Estado 1
    ;S1
    movlw   15		    ;Se mueve 15 a W (-)
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+6  ;Se mueve W de la tabla a Display_Word+6
    movwf   Display_Word+7  ;Se mueve W de la tabla a Display_Word+7
    return
    ;S5
    movlw 10		    ;Se mueve 15 a W (A)
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+6  ;Se mueve W de la tabla a Display_Word+6
    movlw 12		    ;Se mueve 15 a W (C)
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción
    movwf   Display_Word+7  ;Se mueve W de la tabla a Display_Word+7
;-------------------------------------------------------------------------------    
    return

indicador_estado:;(ESTO PERMITE MOSTRAR QUE VIA SE MODIFICA CON LOS LEDS AZULES)
    
    movf    Current_State,w ;Se coloca el valor de Current_State en W
    movwf   PORTE	    ;Se mueve W a PortE
    
    return
    
Via_select:
    ;--------ASIGNAR VALOR A W PARA USAR EN VAR_VAL EN PREPARAR DISPLAYS--------
    btfsc   Nibble_PortA+1,0;Se reivsa el bit 0 de Nibble_PortA+1
    goto    $+2		    ;Salta a las instrucciones Nibble_PortA+1= 0000 0001
    goto    $+24	    ;Salta a revisar el bit 1 de Nibble_PortA+1
    ;INSTRUCCIONES PARA NIBBLE_PORTA+1 = 0000 0001
    btfsc   No_Semaforo,0   ;Se revisa bit 0 de No_Semaforo 
    goto    $+2		    ;Salta a las instrucciones No_Semaforo = 0000 0001
    goto    $+14	    ;Salta a revisar bit 1 de No_Semaforo
   ;INSTRUCCIONES PARA VALOR W CUANDO NO_SEMAFORO = 0000 0001
    btfsc   Nibble_PortA+0,2;Se revisa el bit 2 de Nibble_PortA+0
    goto    $+2		    ;Salta a las instrucciones Nibble_PortA+0= 0000 0100 
    goto    $+3		    ;Salta a revisar el bit 1 de Nibble_PortA+0
    ;INSTRUCCIONES PARA NIBLE_PORTA+0 = 0000 0100
    movf    TV1,w	    ;Se coloca el valor de TV1 en W
    return
    ;INSTRUCCIONES PARA NIBLE_PORTA+0 = 0000 0010 ó 0000 0001
    movlw   4		    ;Se mueve 4 a W
    subwf   TV1,f	    ;Se resta 4 a TV1 (para que el contador separe el teimpo en amarillo)
    movf    TV1,w	    ;Se mueve el resultado a W
    movwf   Var_Val	    ;Se mueve el valor a Var_Val
    movlw   4		    ;Se mueve 4 a W
    addwf   TV1,f	    ;Se suma W a TV1
    movf    Var_Val,w	    ;Se mueve Var_Val a W
    return
    ;INSTRUCCIONES PARA VALOR W CUANDO NO_SEMAFORO = 0000 0010
    btfsc   No_Semaforo,1   ;Se revisa el bit 1 de No_Semaforo
    movf    TV1,w	    ;Se mueve TW1 a W
    ;INSTRUCCIONES PARA VALOR W CUANDO NO_SEMAFORO = 0000 0100
    btfsc   No_Semaforo,2   ;Se revisa el bit 2 de No_Semaforo
    goto    $+2		    ;Salta a las instrucciones para No_Semaforo=0000 0010
    goto    $+3		    ;Salta a revisar el bit 1 de Nibble_PortA+1
    movf    TV1,w	    ;Se mueve TV1 a W
    addwf   TV2,w	    ;Se suma TV2 a W (TV1) y se almacena en W
    ;---------------------------------------------------------------------------
    btfsc   Nibble_PortA+1,1;Se reivsa el bit 1 de Nibble_PortA+1
    goto    $+2		    ;Salta a las instrucciones Nibble_PortA+1= 0000 0001
    goto    $+24	    ;Salta a revisar el bit 1 de Nibble_PortA+1
    
    btfsc   No_Semaforo,1   ;Se revisa bit 1 de No_Semaforo 
    goto    $+2		    ;Salta a las instrucciones No_Semaforo = 0000 0010
    goto    $+14	    ;Salta a revisar bit 2 de No_Semaforo
    
    btfsc   Nibble_PortA+0,2;Se revisa el bit 2 de Nibble_PortA+0
    goto    $+2		    ;Salta a las instrucciones Nibble_PortA+0= 0000 0100 
    goto    $+3		    ;Salta a revisar el bit 1 de Nibble_PortA+0
    movf    TV2,w	    ;Se coloca el valor de TV2 en W
    return
    
    movlw   4		    ;Se mueve 4 a W
    subwf   TV2,f	    ;Se resta 4 a TV2 
    movf    TV2,w	    ;Se mueve el resultado a W
    movwf   Var_Val	    ;Se mueve el valor a Var_Val
    movlw   4		    ;Se mueve 4 a W
    addwf   TV2,f	    ;Se suma W a TV2
    movf    Var_Val,w	    ;Se mueve Var_Val a W
    return
    
    btfsc   No_Semaforo,2   ;Se revisa el bit 1 de No_Semaforo
    movf    TV2,w	    ;Se mueve TW2 a W
    
    btfsc   No_Semaforo,0   ;Se revisa el bit 0 de No_Semaforo
    goto    $+2		    ;Salta a las instrucciones para No_Semaforo=0000 0001
    goto    $+3		    ;Salta a revisar el bit 1 de Nibble_PortA+1
    movf    TV2,w	    ;Se mueve TV2 a W
    addwf   TV3,w	    ;Se suma TV3 a W (TV2) y se almacena en W
    ;--------------------------------------------------------------------------- 
    btfsc   Nibble_PortA+1,2;Se reivsa el bit 2 de Nibble_PortA+1
    goto    $+2		    ;Salta a las instrucciones Nibble_PortA+1= 0000 0010
    goto    $+24	    ;Salta a revisar el bit 1 de Nibble_PortA+1
    
    btfsc   No_Semaforo,2   ;Se revisa bit 2 de No_Semaforo 
    goto    $+2		    ;Salta a las instrucciones No_Semaforo = 0000 0100
    goto    $+14	    ;Salta a revisar bit 0 de No_Semaforo
    
    btfsc   Nibble_PortA+0,2;Se revisa el bit 2 de Nibble_PortA+0
    goto    $+2		    ;Salta a las instrucciones Nibble_PortA+0= 0000 0100
    goto    $+3		    ;Salta a revisar el bit 1 de Nibble_PortA+0
    movf    TV3,w	    ;Se coloca el valor de TV2 en W
    return
    
    movlw   4		    ;Se mueve 4 a W
    subwf   TV3,f	    ;Se resta 4 a TV3 
    movf    TV3,w	    ;Se mueve el resultado a W
    movwf   Var_Val	    ;Se mueve el valor a Var_Val
    movlw   4		    ;Se mueve 4 a W
    addwf   TV3,f	    ;Se suma W a TV3
    movf    Var_Val,w	    ;Se mueve Var_Val a W
    return
    
    btfsc   No_Semaforo,0   ;Se revisa el bit 0 de No_Semaforo
    movf    TV3,w	    ;Se mueve TW3 a W
    
    btfsc   No_Semaforo,1   ;Se revisa el bit 1 de No_Semaforo
    goto    $+2		    ;Salta a las instrucciones para No_Semaforo=0000 0010
    goto    $+3		    ;Salta a revisar el bit 1 de Nibble_PortA+1
    movf    TV3,w	    ;Se mueve TV3 a W
    addwf   TV1,w	    ;Se suma TV1 a W (TV3) y se almacena en W
    return
    
Via_Mod_select:
    ;---------------ASIGNACIÓN DE VALORES A DISPLAY EN MODIFICACION-------------
    btfsc   Current_State,0 ;Se revisa si el Current_State es 0000 0001
    movf    TV1_t,w	    ;Se mueve TV1_t a W
    movwf   Via_Mod	    ;Se mueve W a Via_Mod
    
    btfsc   Current_State,1 ;Se revisa si el Current_State es 0000 0010
    movf    TV2_t,w	    ;Se mueve TV2_t a W
    movwf   Via_Mod	    ;Se mueve W a Via_Mod
    
    btfsc   Current_State,2 ;Se revisa si el Current_State es 0000 0100
    movf    TV3_t,w	    ;Se mueve TV3_t a W
    movwf   Via_Mod	    ;Se mueve W a Via_Mod
    
    btfsc   Current_State,3 ;Se revisa si el Current_State es 0000 1000
    movwf   Via_Mod	    ;Se mueve W a Via_Mod
    
    return
    ;---------------------------------------------------------------------------
Conteo_Reset:
    
    clrf    Delay_1	    ;Se resetea la variable Delay_1
    clrf    Delay_3	    ;Se resetea la variable Delay_3
    
    
    movlw   255		    ;Se coloca en W el valor calculado para TMR1H
    movwf   TMR1H	    ;Se mueve W a TMR1H
    movlw   248		    ;Se coloca en W el valor calculado para TMR1L
    movwf   TMR1L	    ;Se mueve W a TMR1L
    
    movlw   1		    ;Se mueve 0000 0001 a W
    movwf   Nibble_PortA+0  ;Se mueve W a Nibble_PortA+0
    movwf   Nibble_PortA+1  ;Se mueve W a Nibble_PortA+1
    
    return
    
Secuencia_Reset:
    
    clrf    PORTC	    ;Se coloca PORTC en 0000 0000
    clrf    PORTA	    ;Se coloca PORTA en 0000 0000
    ;SE ENCIENDEN LAS LUCES ROJAS DE LOS SEMÁFOROS 
    bsf	    PORTA,6	    ;Se enciende el bit 6 de PORTA
    bsf	    PORTA,7	    ;Se enciende el bit 7 de PORTA
    bsf	    PORTC,7	    ;Se enciende el bit 7 de PORTC
    clrf    Delay_2	    ;Se resetea la variable Delay_2
    clrf    Delay_4	    ;Se resetea la variable Delay_4
    call    delay_big	    ;Se llama a la funcion delay_big
    clrf    PORTA	    ;Se coloca PORTA en 0000 0000
    bcf	    PORTC,7	    ;Se apaga el bit 7 de PORTC
    ;SE ENCIENDEN LAS LUCES AMARILLAS DE LOS SEMÁFOROS 
    bsf	    PORTA,3	    ;Se enciende el bit 3 de PORTA
    bsf	    PORTA,4	    ;Se enciende el bit 4 de PORTA
    bsf	    PORTA,5	    ;Se enciende el bit 5 de PORTA
    clrf    Delay_2	    ;Se resetea la variable Delay_2
    clrf    Delay_4	    ;Se resetea la variable Delay_4
    call    delay_big	    ;Se llama a la funcion delay_big
    call    Conteo_Reset    ;Se llama a la funcion Conteo_Reset
    clrf    PORTA	    ;Se coloca PORTA en 0000 0000
    ;SE ENCIENDEN LAS LUCES ROJAS DE LOS SEMÁFOROS
    bsf	    PORTA,6	    ;Se enciende el bit 6 de PORTA
    bsf	    PORTA,7	    ;Se enciende el bit 7 de PORTA
    bsf	    PORTC,7	    ;Se enciende el bit 7 de PORTC
    clrf    Delay_2	    ;Se resetea la variable Delay_2
    clrf    Delay_4	    ;Se resetea la variable Delay_4
    call    delay_big	    ;Se llama a la funcion delay_big
    
    movlw   0		    ;Se coloca 0000 0000 en W
    movwf   Current_State   ;Se mueve W a Current_State
    
    return    

Led_Matrix:
    ;Nibble_PortA+1 = semaforos 1, 2 y 3
    ;Nibble_PortA+0 = verde,verde titilante, amarillo
    
    /*
    swapf   Nibble_PortA+1,w
    andlw   0xf0	    
    movwf   PORTA
    movf    Nibble_PortA+0,w
    andlw   0x0f
    addwf   PORTA, F
    
    
    Inicialmente se planteó el proyecto empleando lógica combinacional cuya 
    prohibición no estaba implícita en el documento. El fragmento de código 
    anterior movia los valores de Nibble_PortA+1 y Nibble_PortA+0 a PORTA donde
    en base a la combinacion dada la lógica combinacional encendía las leds 
    pertinentes. Para evitar modificaciones significativas al código, pues hizo 
    falta únicamente la modificación de esta función, se sustituyó la lógica 
    combinacional con el código a continuación aún utilizando las combinaciones
    ya funcionales de las variables Nibble_PortA+1 y Nibble_PortA+0
    */
    
    
    
    ;----------------------MANEJO DE LUCES SEMÁFORO-----------------------------
    ;APAGAR LUCES ANTERIORES Y ENCENDER LUCES FIJAS (ROJOS)
    btfsc   Nibble_PortA+1,0;Se revisa el bit 0 de Nibble_PortA+1	
    goto    $+2		    ;Se salta a instrucciones Nibble_PortA+1 = 0000 0001
    goto    $+18	    ;Se salta a revisar el bit 1 de Nibble_PortA+1
    bcf	    PORTC, 7	    ;Apaga el bit 7 de PORTC (ROJO, SEMAFORO 1)
    bcf	    PORTA, 3	    ;Apaga el bit 3 de PORTA (AMARILLO, SEMAFORO 3)
    bsf	    PORTA, 7	    ;Enciende el bit 7 de PORTA (ROJO, SEMAFORO 2)
    bsf	    PORTA, 6	    ;Enciende el bit 6 de PORTA (ROJO, SEMAFORO 3)
    ;ENCEINDE LUZ VERDE CONSTANTE
    btfsc   Nibble_PortA+0,0;Se revisa el bit 0 de Nibble_PortA+0
    bsf	    PORTA, 2	    ;Enciende el bit 2 de PORTA (VERDE, SEMAFORO 1)
    ;ENCIENDE/APAGA LUZ VERDE SEGUN TOGGLE (VERDE TITILANTE)
    btfsc   Nibble_PortA+0,1;Se revisa el bit 1 de Nibble_PortA+0
    goto    $+2		    ;Salta a instrucciones Nibble_PortA+0 = 0000 0001
    goto    $+4		    ;Salta a revisar el bit 2 de Nibble_PortA+0
    bcf	    PORTA, 2	    ;Apaga el bit 2 de PORT1 (VERDE, SEMAFORO 1)
    btfsc   toggle,1	    ;Se revis a si la señal toggle esta encendida
    bsf	    PORTA, 2	    ;Enciende el bit 2 de PORTA (VERDE, SEMAFORO 1)
    ;ENCIENTE LUZ AMARILLA
    btfsc   Nibble_PortA+0,2;Se revisa el bit 2 de Nibble_PortA+0
    goto    $+2		    ;Salta a instrucciones Nibble_PortA+0 = 0000 0010
    goto    $+3		    ;Se salta a revisar el bit 1 de Nibble_PortA+1
    bcf	    PORTA, 2	    ;Apaga el bit 2 de PORT1 (VERDE, SEMAFORO 1)
    bsf	    PORTA, 5	    ;Enciende el bit 5 de PORTA (AMARILLO, SEMAFORO 1)
    ;---------------------------------------------------------------------------
    btfsc   Nibble_PortA+1,1;Se revisa el bit 1 de Nibble_PortA+1
    goto    $+2		    ;Se salta a instrucciones Nibble_PortA+1 = 0000 0010
    goto    $+18	    ;Se salta a revisar el bit 2 de Nibble_PortA+1
    bcf	    PORTA, 5	    ;Apaga el bit 5 de PORTA (AMARILLO, SEMAFORO 1)
    bcf	    PORTA, 7	    ;Apaga el bit 7 de PORTA (ROJO, SEMAFORO 2)
    bsf	    PORTC, 7	    ;Enciende el bit 7 de PORTC (ROJO, SEMAFORO 1)
    bsf	    PORTA, 6	    ;Enciende el bit 6 de PORTA (ROJO, SEMAFORO 3)
    btfsc   Nibble_PortA+0,0;Se revisa el bit 0 de Nibble_PortA+0
    bsf	    PORTA, 1	    ;Enciende el bit 1 de PORTA (VERDE, SEMAFORO 2)
    btfsc   Nibble_PortA+0,1;Se revisa el bit 1 de Nibble_PortA+0
    goto    $+2		    ;Salta a instrucciones Nibble_PortA+0 = 0000 0001
    goto    $+4		    ;Salta a revisar el bit 2 de Nibble_PortA+0
    bcf	    PORTA, 1	    ;Apaga el bit 1 de PORTA (VERDE, SEMAFORO 2)
    btfsc   toggle,1	    ;Se revis a si la señal toggle esta encendida
    bsf	    PORTA, 1	    ;Enciende el bit 1 de PORTA (VERDE, SEMAFORO 2)
    btfsc   Nibble_PortA+0,2;Se revisa el bit 2 de Nibble_PortA+0
    goto    $+2		    ;Salta a instrucciones Nibble_PortA+0 = 0000 0010
    goto    $+3		    ;Se salta a revisar el bit 2 de Nibble_PortA+1
    bcf	    PORTA, 1	    ;Apaga el bit 1 de PORTA (VERDE, SEMAFORO 2)
    bsf	    PORTA, 4	    ;Enciende el bit 4 de PORTA (AMARILLO, SEMAFORO 2)
    
    btfsc   Nibble_PortA+1,2;Se revisa el bit 2 de Nibble_PortA+1
    goto    $+2		    ;Se salta a instrucciones Nibble_PortA+1 = 0000 0100
    goto    $+18	    ;Se salta a finalizar la interrupción
    bcf	    PORTA, 4	    ;Apaga el bit 4 de PORTA (AMARILLO, SEMAFORO 2)
    bcf	    PORTA, 6	    ;Apaga el bit 6 de PORTA (ROJO, SEMAFORO 3)
    bsf	    PORTA, 7	    ;Enciende el bit 7 de PORTA (ROJO, SEMAFORO 2)
    bsf	    PORTC, 7	    ;Enciende el bit 7 de PORTC (ROJO, SEMAFORO 1)
    btfsc   Nibble_PortA+0,0;Se revisa el bit 0 de Nibble_PortA+0
    bsf	    PORTA, 0	    ;Enciende el bit 0 de PORTA (VERDE, SEMAFORO 3)
    btfsc   Nibble_PortA+0,1;Se revisa el bit 1 de Nibble_PortA+0
    goto    $+2		    ;Salta a instrucciones Nibble_PortA+0 = 0000 0001
    goto    $+4		    ;Salta a revisar el bit 2 de Nibble_PortA+0
    bcf	    PORTA, 0	    ;Apaga el bit 0 de PORTA (VERDE, SEMAFORO 3)
    btfsc   toggle,1	    ;Se revis a si la señal toggle esta encendida
    bsf	    PORTA, 0	    ;Enciende el bit 0 de PORTA (VERDE, SEMAFORO 3)
    btfsc   Nibble_PortA+0,2;Se revisa el bit 2 de Nibble_PortA+0
    goto    $+2		    ;Salta a instrucciones Nibble_PortA+0 = 0000 0010
    goto    $+3		    ;Se salta a finalizar la interrupción
    bcf	    PORTA, 0	    ;Apaga el bit 0 de PORTA (VERDE, SEMAFORO 3)
    bsf	    PORTA, 3	    ;Enciende el bit 3 de PORTA (AMARILLO, SEMAFORO 3)
    
    return
    
delay_big:
    movlw   198		    ;Se coloca un valor calculado en W
    movwf   Delay_2	    ;Se mueve W a la variable Delay_2
    call    delay_small	    ;Se llama a la función delay_small
    decfsz  Delay_2, 1	    ;Se decrementa la variable Delay_2 a menos que sea 0
    goto    $-2		    ;Regresa 2 instrucciones
    return
    
delay_small:	
    movlw   248		    ;Se coloca un valor calculado en W
    movwf   Delay_4	    ;Se mueve W a la variable Delay_4
    decfsz  Delay_4, 1	    ;Se decrementa la variable Delay_2 a menos que sea 0
    goto    $-1		    ;Regresa 1 instrucción
    return
    
Print_PortC:
    
     /*
    Otro cambio realizado por la imposibilidad de emplear compuertas lógicas fue
    la adición de la presente función, donde se copia bit por bit el valor que
    tiene la variable Display_Word. Esto porque una de las leds del semáforo se 
    encuentra en PORTC por lo que no se pueden almacenar valores en dicha 
    variable sin afectar el desempeño de la luz del semáforo determinada por la
    salida PORTC, 7
    */
    
    
    btfsc   PORTC_t,0	    ;Se revisa si el bit 0 de PORTC_t esta encendido
    goto    $+2		    ;Salta a las instricciones si el bit 0 está encendido
    goto    $+3		    ;Salta a apagar el bit 0 de PORTC si está apagado
    bsf	    PORTC,0	    ;Enciende el bit 0 de PORTC
    goto    $+2		    ;Salta a revisar el bit 1 de PORTC_t
    bcf	    PORTC,0	    ;Apaga el bit 0 de PORTC
    
    btfsc   PORTC_t,1	    ;Se revisa si el bit 1 de PORTC_t esta encendido
    goto    $+2		    ;Salta a las instricciones si el bit 1 está encendido
    goto    $+3		    ;Salta a apagar el bit 1 de PORTC si está apagado
    bsf	    PORTC,1	    ;Enciende el bit 1 de PORTC
    goto    $+2		    ;Salta a revisar el bit 2 de PORTC_t
    bcf	    PORTC,1	    ;Apaga el bit 1 de PORTC
    
    btfsc   PORTC_t,2	    ;Se revisa si el bit 2 de PORTC_t esta encendido
    goto    $+2		    ;Salta a las instricciones si el bit 2 está encendido
    goto    $+3		    ;Salta a apagar el bit 2 de PORTC si está apagado
    bsf	    PORTC,2	    ;Enciende el bit 2 de PORTC
    goto    $+2		    ;Salta a revisar el bit 3 de PORTC_t
    bcf	    PORTC,2	    ;Apaga el bit 2 de PORTC
    
    btfsc   PORTC_t,3	    ;Se revisa si el bit 3 de PORTC_t esta encendido
    goto    $+2		    ;Salta a las instricciones si el bit 3 está encendido
    goto    $+3		    ;Salta a apagar el bit 3 de PORTC si está apagado
    bsf	    PORTC,3	    ;Enciende el bit 3 de PORTC
    goto    $+2		    ;Salta a revisar el bit 4 de PORTC_t
    bcf	    PORTC,3	    ;Apaga el bit 3 de PORTC
    
    btfsc   PORTC_t,4	    ;Se revisa si el bit 4 de PORTC_t esta encendido
    goto    $+2		    ;Salta a las instricciones si el bit 4 está encendido
    goto    $+3		    ;Salta a apagar el bit 4 de PORTC si está apagado
    bsf	    PORTC,4	    ;Enciende el bit 4 de PORTC
    goto    $+2		    ;Salta a revisar el bit 5 de PORTC_t
    bcf	    PORTC,4	    ;Apaga el bit 4 de PORTC
    
    btfsc   PORTC_t,5	    ;Se revisa si el bit 5 de PORTC_t esta encendido
    goto    $+2		    ;Salta a las instricciones si el bit 5 está encendido
    goto    $+3		    ;Salta a apagar el bit 5 de PORTC si está apagado
    bsf	    PORTC,5	    ;Enciende el bit 5 de PORTC
    goto    $+2		    ;Salta a revisar el bit 6 de PORTC_t
    bcf	    PORTC,5	    ;Apaga el bit 5 de PORTC
    
    btfsc   PORTC_t,6	    ;Se revisa si el bit 6 de PORTC_t esta encendido
    goto    $+2		    ;Salta a las instricciones si el bit 6 está encendido
    goto    $+3		    ;Salta a apagar el bit 6 de PORTC si está apagado
    bsf	    PORTC,6	    ;Enciende el bit 6 de PORTC
    goto    $+2		    ;Salta a finalizar la interrupción
    bcf	    PORTC,6	    ;Apaga el bit 6 de PORTC
    
    
    return
END


