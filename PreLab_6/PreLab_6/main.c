//***************************************************************************
// Universidad del Valle de Guatemala
// IE2023: Programación de Microcontroladores 
// Hardware: ATMEGA328PB
// Created: 20/04/2025
// Descripción: Comunicación UART con ATMEGA328PB
//                 Enviar y recibir un carácter, mostrarlo en el PORTB y PORTD

//***************************************************************************
// PreLab_6
//***************************************************************************

#define F_CPU 16000000UL  // Frecuencia del reloj: 16MHz
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

void initUART(void);
void writeUART(char caracter);
void writeTextUART(char* texto);
void asciiVal(void);
void MenuL0(void);

uint8_t StateAscii = 0;
char buffLast = '0';

volatile char bufferRX;

int main(void)
{
    initUART();
    
    sei();
    DDRD = 0xFF;
    DDRB = 0xFF;
    
    MenuL0();
    PORTD = 0;
    PORTB = 0;
    
    while (1){
        asciiVal();
    }    
}

void initUART(void){
    //Rx y Tx
    DDRD &= ~(1<<DDD0);
    DDRD |= (1<<DDD1);
    
    //Fastmode
    //Configurar A
    UCSR0A = 0;
    UCSR0A |= (1<<U2X0);
    
    //Configurar B /ISR Rx /Habilitar RX y TX
    UCSR0B = 0;
    UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
    
    //Frame de 8 bits, sin paridad, 1 bit stop
    UCSR0C = 0;
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
    
    //Baudrate9600
    UBRR0 = 207;
}

void writeUART(char caracter){
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = caracter;
}
    
void writeTextUART(char* texto){
    uint8_t i;
    for (i=0; texto[i] != '\0'; i++){
        while(!(UCSR0A&(1<<UDRE0)));
        UDR0 = texto[i];
    }
}

ISR(USART0_RX_vect){
    bufferRX = UDR0;
    if(StateAscii >= 1) {
        PORTB |= (1<<PB5);
        buffLast = bufferRX;
    }
    while(!(UCSR0A&(1<<UDRE0)));
    UDR0 = bufferRX;
    if (StateAscii >= 1){
        StateAscii++;
        if (StateAscii >= 2){
            StateAscii = 0;
            MenuL0();
        }
    }
    if (bufferRX == '1'){
        if (StateAscii == 0){
            writeTextUART("\nAscci\n");
            writeTextUART("\nEscribe tu código Ascii Bombardini Gussini:\n");
            StateAscii = 1;
            PORTD = 0;
            PORTB = 0;
        }
    }
}

void asciiVal(void) {
    // Asegúrate de que buffLast esté en el rango de 0 a 255
    if (buffLast < 0) {
        buffLast = 0; // Limitar a 0 si es negativo
    } else if (buffLast > 255) {
        buffLast = 255; // Limitar a 255 si es mayor
    }

    // Desplazamiento de bits
    PORTD = (buffLast << 2) & 0xFC; // Muestra los bits 2-7 en PORTD (PD2 a PD7)
    PORTB = (buffLast >> 6) & 0x03; // Muestra los bits 6-7 en PORTB (PB0 y PB1)
}

void MenuL0(){
    writeTextUART("\nElige una opción:\n");
    writeTextUART("1. Enviar Ascii\n");
}

