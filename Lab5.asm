;Archivo:		Lab5.s
;Dispositivo:		PIC16F887
;Autor;			Fernando Jose Caceros Morales
;Compilador:		pic-as (v2.31) MPLABX V5.40
;
;Programa:		
;Hardware:		Pushbuttons en puerto B, Leds en Puerto C, 7seg en puerto D 
;
;Creado:		2 marzo, 2021
;Ultima modificacion:	2 marzo, 2021
    
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
    Cont_Cent:		DS 1 ; 1 byte
    Cont_Dec:		DS 1 ; 1 byte
    var:		DS 1 ; 1 byte
    PORTD_temp:		DS 1 ; 1 byte
    nibble:		DS 2 ; 2 bytes
    display_var:	DS 5 ; 5 bytes
    
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
    retlw   71h	    ; F
    
main:
    call    config_IO	    ;inputs PORTB, outputs PORTA, PORTC, PORTD
    call    Tm0_config	    ;Configuración Timer0
    call    reloj_config    ;Configuración del reloj
    call    config_iocb	    ;Configuración del Interrupt On Change 
    call    config_interrup ;Configuración de las interrupciones
    banksel PORTA	    ;Se selecciona el banco 0
    clrf    PORTA	    ;Se inicializa el puerto A
    clrf    PORTD	    ;Se inicializa el puerto D
    movlw   3Fh		    ;Se coloca 0011 1111 en W
    movwf   PORTC	    ;Se mueve W al puerto C
    
    ;se encienden y apagan todos los displays para inicializar todos con 0
    
    bsf	    PORTD, 0	    ;Puerto para display hex 1	    
    bcf	    PORTD, 0
    bsf	    PORTD, 1	    ;Puerto para display hex 0
    bcf	    PORTD, 1
    bsf	    PORTD, 3	    ;Puerto para display decimal 2
    bcf	    PORTD, 3
    bsf	    PORTD, 4	    ;Puerto para display decimal 1
    bcf	    PORTD, 4
    bsf	    PORTD, 5	    ;Puerto para display decimal 0
    bcf	    PORTD, 5 
    movlw   01h		    ;Se coloca 0000 0001 en W
    movwf   PORTD_temp	    ;Se mueve W a la variable temporal del puerto D
    clrf    Cont_Cent	    ;Se inicializa la variable contador de centena
    clrf    Cont_Dec	    ;Se inicializa la variable contador de decenas
    
    
    
;********************************loop proncipal*********************************
    
loop:
    call separar_nibbles
    call preparar_display
    call contador_centenas
    goto loop
    
;**********************************subrutina************************************

config_IO: 
    banksel ANSEL	    ;Se selecciona el banco 3
    clrf    ANSEL	    ;Se colocan los pines como digitales
    clrf    ANSELH
    
    banksel TRISA	    ;Se selecciona el banco 1
    bsf	    TRISB, 0	    ;PORTB, pines 0 y 1 como entrada
    bsf	    TRISB, 1
    clrf    TRISA	    ;PORT A, PORTC y PORTD como salidas
    clrf    TRISC   
    clrf    TRISD
    bcf	    OPTION_REG, 7   ;habilita pull-ups
    bsf	    WPUB, 0	    ;Incrementar con PB en 0
    bsf	    WPUB, 1	    ;Decrementar con PB en 1
    return

Tm0_config:
    banksel TRISA	    ;Se selecciona el banco 1
    bcf	    T0CS	    ;Se selecciona el reloj interno
    bcf	    PSA		    ;Se habilita la asignacion de prescaler al Timer0
    bcf	    PS2
    bsf	    PS1
    bcf	    PS0		    ;Prescaler 1 a 8
    banksel PORTA	    ;Se selecciona el banco 0
    movlw   125		    ;Se coloca el numero a cargarse hallado con la formula
    movwf   TMR0	    ;Se mueve ese valor al Timer0
    bcf	    T0IF	    ;Se apaga la bandera de interrupcion del Timer0
    return

reloj_config:	
    banksel TRISA	    ;Se selecciona el banco 1
    
    ;Se selecciona una velocidad de reloj 
    bcf	    IRCF2	    
    bsf	    IRCF1
    bsf	    IRCF0	    ; reloj a 500KHz 
    return

config_interrup:
    banksel TRISA	    ;Se selecciona el banco 1
    bsf	    GIE		    ;Se habilita cualquier tipo de interrupción
    
    bsf	    RBIE	    ;Se habilita la interrupción del puerto B
    bcf	    RBIF	    ;Se apaga la bandera de interrupcion del puerto B
    
    bsf	    T0IE	    ;Se habilita la interrupción del Timer0
    bcf	    T0IF	    ;Se apaga la bandera de interrupcion del Timer0
    return

config_iocb:
    banksel TRISA	    ;Se selecciona el banco 1
    bsf	    IOCB, 0	    ;Se coloca una interrupcion el pin 0 del puerto B
    bsf	    IOCB, 1	    ;Se coloca una interrupcion el pin 0 del puerto B
    
    banksel PORTA	    ;Se selecciona el banco 0
    movf    PORTB, W	    ;Se miebe el valor del puerto B a W
    bcf	    RBIF	    ;Se apaga la bandera de interrupción del puerto B
    return

;***************************rutina de interrupciones****************************

int_OCB:
    btfss   PORTB, 0	    ;Se revisa el pin 0 del puerto B 
    incf    PORTA	    ;Se incrementa el valor del puerto A
    btfss   PORTB, 1	    ;Se revisa el pin 1 del puertto B
    decf    PORTA	    ;Se decrementa el valor del puerto A
    bcf	    RBIF	    ;Se apaga la bandera de interrupción del puerto B
    return

int_tm0:
    banksel PORTA	    ;Se selecciona el banco 0 
    movlw   125		    ;Se coloca el numero a cargarse hallado con la formula
    movwf   TMR0	    ;Se mueve ese valor al Timer0
    bcf	    T0IF	    ;Se apaga la bandera de interrupción del Tiemer0
    btfsc   PORTD_temp,0    ;Se revisa si esta encendido el pin 0 en el puerto D
    goto    display_1	    ;Se direcciona a la seccion de interrupcion display_1
    btfsc   PORTD_temp,1    ;Se revisa si esta encendido el pin 1 en el puerto D
    goto    display_2	    ;Se direcciona a la seccion de interrupcion display_2
    btfsc   PORTD_temp,2    ;Se revisa si esta encendido el pin 2 en el puerto D
    goto    display_4	    ;Se direcciona a la seccion de interrupcion display_4
    btfsc   PORTD_temp,3    ;Se revisa si esta encendido el pin 3 en el puerto D
    goto    display_5	    ;Se direcciona a la seccion de interrupcion display_5
    btfsc   PORTD_temp,4    ;Se revisa si esta encendido el pin 4 en el puerto D
    goto    display_6	    ;Se direcciona a la seccion de interrupcion display_6
display_1:
    clrf    PORTD	    ;Se coloca en 0 el puerto D
    movf    display_var, W  ;Se coloca el valor del primer nibble en W
    movwf   PORTC	    ;Se mueve ese valor al puerto C
    bsf	    PORTD, 0	    ;Se enciende en el puerto D el pin 0
    goto    next_display    ;Se direcciona a la seccion de interrupcion next_display
display_2:
    clrf    PORTD	    ;Se coloca en 0 el puerto D
    movf    display_var+1, W;Se coloca el valor del segundo nibble en W
    movwf   PORTC	    ;Se mueve ese valor al puerto C
    bsf	    PORTD, 1	    ;Se enciende en el puerto D el pin 1
    goto    next_display    ;Se direcciona a la seccion de interrupcion next_display
display_4:
    clrf    PORTD	    ;Se coloca en 0 el puerto D
    movf    display_var+2, W;Se coloca el valor del tercer nibble en W
    movwf   PORTC	    ;Se mueve ese valor al puerto C
    bsf	    PORTD, 3	    ;Se enciende en el puerto D el pin 3
    goto    next_display    ;Se direcciona a la seccion de interrupcion next_display
display_5:
    clrf    PORTD	    ;Se coloca en 0 el puerto D
    movf    display_var+3, W;Se coloca el valor del cuarto nibble en W
    movwf   PORTC	    ;Se mueve ese valor al puerto C
    bsf	    PORTD, 4	    ;Se enciende en el puerto D el pin 4
    goto    next_display    ;Se direcciona a la seccion de interrupcion next_display
display_6:
    clrf    PORTD	    ;Se coloca en 0 el puerto D
    movf    display_var+4, W;Se coloca el valor del quinto nibble en W
    movwf   PORTC	    ;Se mueve ese valor al puerto C
    bsf	    PORTD, 5	    ;Se enciende en el puerto D el pin 5
    goto    next_display    ;Se direcciona a la seccion de interrupcion next_display
next_display:
	bcf	CARRY	    ;Se apaga la señal carry
	btfss   PORTD_temp,5;Se revisa si el bit 5 del puerto D temporal está encendido
	goto	$+3	    ;Se salta 3 instrucciones
	movlw	01h	    ;Se coloca 0000 0001 en W
	movwf	PORTD_temp  ;Se mueve W al puerto D temporal
	rlf	PORTD_temp,F;Se coloca la señal carry (0) a la derecha moviendo el pin encendido 
	return

    
;*********************************subrutinas************************************
	
	
separar_nibbles:
    movf    PORTA, w	    ;Se mueve el valor del puerto A a W
    andwf   0x0f	    ;Se compara con 0000 1111 para solo obtener el nibble menos significativo
    movwf   nibble+1	    ;Se almacena en la variable nibble en la segunda posoción
    swapf   PORTA, w	    ;Se mueve el valor del puerto A a W con los nibbles cambiados
    andwf   0x0f	    ;Se compara con 0000 1111 para solo obtener el nibble menos significativo
    movwf   nibble	    ;Se almacena en la variable nibble en la primera posoción
    return
    
preparar_display:
    movf    nibble, w	    ;Se coloca el valor de nibble 0 en W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción de digito a display
    movwf   display_var	    ;Se mueve el valor obtenido de la tabla a la variable del display
    movf    nibble+1, w	    ;Se coloca el valor de nibble 1 en W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción de digito a display
    movwf   display_var+1   ;Se mueve el valor obtenido de la tabla a la variable del display
    return

contador_centenas:
    clrf    Cont_Cent	    ;Se inicializa la variable 
    bcf	    CARRY	    ;Se apaga la bandera CARRY
    movf    PORTA, w	    ;Se mueve el valor del puerto A a W
    movwf   var		    ;Se mueve el valor de W a la variable var
    movlw   100		    ;Se pone en W un valor de 100 (por tratarse de centenas)
    subwf   var, F	    ;Se resta 100 a var y se guarde el resultado en var
    incf    Cont_Cent	    ;Se incrementa el contador de centenas
    btfsc   CARRY	    ;Se revisa si la bandera de carry se apaga (el numero es negativo)
    goto    $-3		    ;Se regresa 3 instrucciones en el código
    decf    Cont_Cent	    ;Se decrementa el contador de centenas
    addwf   var		    ;Se suma 100 a var 
    movf    Cont_Cent,w	    ;Se mueve el valor del contador a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción de digito a display
    movwf   display_var+2   ;Se mueve el valor obtenido de la tabla a la variable del display
    call    contador_decenas;Se llama a la subrutina contador_decenas
    return
    
contador_decenas:
    clrf    Cont_Dec	    ;Se inicializa la variable 
    bcf	    CARRY	    ;Se apaga la bandera CARRY
    movlw   10		    ;Se pone en W un valor de 10 (por tratarse de decenas)
    subwf   var		    ;Se resta 10 a var y se guarde el resultado en var
    incf    Cont_Dec	    ;Se incrementa el contador de decenas
    btfsc   CARRY	    ;Se revisa si la bandera de carry se apaga (el numero es negativo)
    goto    $-3		    ;Se regresa 3 instrucciones en el código
    decf    Cont_Dec	    ;Se decrementa el contador de decenas
    addwf   var		    ;Se suma 10 a var 
    movf    Cont_Dec,w	    ;Se mueve el valor del contador a W
    call    siete_seg	    ;Se traslada el valor a la tabla de traducción de digito a display
    movwf   display_var+3   ;Se mueve el valor obtenido de la tabla a la variable del display
    
    movf    var, w	    ;Se mueve el valor de var a W
    call    siete_seg	    
    movwf   display_var+4
    
    return

    
END