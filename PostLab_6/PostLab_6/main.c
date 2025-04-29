//***************************************************************************
// Universidad del Valle de Guatemala
// IE2023: Programaci�n de Microcontroladores 
// Hardware: ATMEGA328PB
// Created: 20/04/2025
// Descripci�n: Comunicaci�n UART con ATMEGA328PB
//                 Parte 1: Enviar un car�cter al PC
//                 Parte 2: Recibir un car�cter del PC y mostrarlo en el PORTB
//***************************************************************************
// PostLab 6
//****************************************************************************/

// Define la frecuencia del reloj del microcontrolador (16MHz)
#define F_CPU 16000000UL  

// Inclusi�n de bibliotecas necesarias
#include <avr/io.h>        // Biblioteca est�ndar para E/S de AVR
#include <util/delay.h>    // Funciones de retardo
#include <avr/interrupt.h> // Manejo de interrupciones
#include <stdio.h>         // Funciones est�ndar de E/S (para sprintf)

// Declaraci�n de prototipos de funciones
void initUART (void);           // Inicializaci�n de comunicaci�n UART
void writeUART (char caracter); // Env�a un solo car�cter por UART
void writeTextUART (char* texto); // Env�a una cadena de texto por UART
void asciiVal(void);            // Muestra el valor ASCII en los puertos
void MenuL0(void);              // Muestra el men� principal
void ADC_init(void);            // Inicializa el conversor anal�gico-digital
uint16_t adcRead(uint8_t);      // Lee un valor del ADC de un canal espec�fico

// Variables globales
int adcValue1 = 0;        // Almacena el valor le�do del ADC
uint8_t StateAscii = 0;   // Estado para la funci�n de mostrar caracteres ASCII
uint8_t StatePot = 0;     // Estado para la funci�n de leer potenci�metro
char buffLast = '0';      // �ltimo car�cter recibido por UART
char buffer[10];          // Buffer para convertir valores num�ricos a texto

// Variable para almacenar caracteres recibidos por interrupci�n
volatile char bufferRX;   // El modificador 'volatile' indica que esta variable puede cambiar en cualquier momento (por la ISR)

int main(void)
{
    // Inicializaci�n de m�dulos
    initUART();    // Configura la comunicaci�n UART
    ADC_init();    // Configura el ADC
    
    sei();         // Habilita las interrupciones globales (Set Interrupt Flag)
    DDRD = 0xFF;   // Configura todo el puerto D como salida
    DDRB = 0xFF;   // Configura todo el puerto B como salida
    
    MenuL0();      // Muestra el men� principal por el puerto serie
    PORTD = 0;     // Inicializa el puerto D en bajo
    PORTB = 0;     // Inicializa el puerto B en bajo
    
    // Bucle principal del programa
    while (1){
        asciiVal();    // Actualiza la visualizaci�n del valor ASCII en los puertos
        
        // Si est� activado el modo de lectura del potenci�metro
        if (StatePot == 1){
            adcValue1 = adcRead(5);  // Lee el valor del potenci�metro en el canal 5
            
            // Convierte el valor entero del ADC en una cadena de caracteres
            sprintf(buffer, "%d", adcValue1);
            
            // Env�a el valor convertido por el puerto serie
            writeTextUART(buffer);
            
            // Env�a un salto de l�nea
            writeTextUART("\n");
            
            // Espera 1 segundo antes de la siguiente lectura
            _delay_ms(1000);
        }
    }    
}

void initUART(void){
    // Configuraci�n de pines Rx (PD0) y Tx (PD1)
    DDRD &= ~(1<<DDD0);  // Configura PD0 (RX) como entrada
    DDRD |= (1<<DDD1);   // Configura PD1 (TX) como salida
    
    // Configuraci�n del registro UCSR0A
    UCSR0A = 0;                // Limpia el registro
    UCSR0A |= (1<<U2X0);       // Activa el modo de velocidad doble (Fast mode)
    
    // Configuraci�n del registro UCSR0B
    UCSR0B = 0;                // Limpia el registro
    UCSR0B |= (1<<RXCIE0);     // Habilita la interrupci�n de recepci�n
    UCSR0B |= (1<<RXEN0);      // Habilita la recepci�n
    UCSR0B |= (1<<TXEN0);      // Habilita la transmisi�n
    
    // Configuraci�n del formato de frame: 8 bits de datos, sin paridad, 1 bit de parada
    UCSR0C = 0;                // Limpia el registro
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);  // Configura 8 bits de datos
    
    // Configuraci�n del baudrate a 9600 bps (con U2X0=1)
    UBRR0 = 207;               // Valor calculado para 9600 bps con U2X0=1 y F_CPU=16MHz
}

void writeUART(char caracter){
    // Espera hasta que el buffer de transmisi�n est� vac�o
    while(!(UCSR0A & (1<<UDRE0)));
    // Coloca el car�cter en el registro de datos para transmitir
    UDR0 = caracter;
}
    
void writeTextUART(char* texto){
    uint8_t i;
    // Recorre cada car�cter hasta encontrar el terminador '\0'
    for (i=0; texto[i] != '\0'; i++){
        // Espera a que el buffer de transmisi�n est� vac�o
        while(!(UCSR0A&(1<<UDRE0)));
        // Env�a el car�cter actual
        UDR0 = texto[i];
    }
}

// Rutina de servicio de interrupci�n para recepci�n UART
ISR(USART0_RX_vect){
    bufferRX = UDR0;  // Lee el car�cter recibido del registro de datos
    
    // Si estamos en el modo ASCII (esperando un car�cter)
    if(StateAscii >= 1) {
        PORTB |= (1<<PB5);  // Enciende el LED en PB5 para indicar recepci�n
        buffLast = bufferRX; // Guarda el car�cter recibido para mostrarlo en los puertos
    }
    
    // Eco: devuelve el car�cter recibido por el puerto serie
    while(!(UCSR0A&(1<<UDRE0)));
    UDR0 = bufferRX;
    
    // Manejo del estado ASCII
    if (StateAscii >= 1){
        StateAscii++;  // Incrementa el estado
        if (StateAscii >= 2){  // Si ya recibimos un car�cter
            StateAscii = 0;    // Volvemos al men� principal
            MenuL0();          // Muestra el men�
        }
    }
    
    // Si estamos en modo lectura de potenci�metro y el usuario presiona '0'
    if (StatePot == 1){
        if (bufferRX == '0'){
            StatePot = 0;      // Desactiva el modo potenci�metro
            MenuL0();          // Vuelve al men� principal
        }
    }
    
    // Manejo de las opciones del men�
    if (bufferRX == '1'){  // Opci�n 1: Leer potenci�metro
        if (StateAscii == 0){  // Si estamos en el men� principal
            writeTextUART("\nValor Potenciometro\n");
            writeTextUART("Escribir 0 para volver\n");
            StatePot = 1;      // Activa el modo potenci�metro
        }
    } else if (bufferRX == '2'){  // Opci�n 2: Modo ASCII
        if (StateAscii == 0){  // Si estamos en el men� principal
            writeTextUART("\nAscci\n");
            writeTextUART("\nEscribe tu c�digo Ascii:\n");
            StateAscii = 1;    // Activa el modo ASCII
            PORTD = 0;         // Limpia el puerto D
            PORTB = 0;         // Limpia el puerto B
        }
    }
}

void asciiVal(void) {
    // Asegura que buffLast est� en el rango de 0 a 255
    if (buffLast < 0) {
        buffLast = 0;          // Limita a 0 si es negativo
    } else if (buffLast > 255) {
        buffLast = 255;        // Limita a 255 si es mayor
    }

    // Muestra el valor ASCII en los puertos utilizando desplazamiento de bits
    PORTD = (buffLast << 2) & 0xFC;  // Muestra los bits 2-7 en PORTD (PD2 a PD7)
    PORTB = (buffLast >> 6) & 0x03;  // Muestra los bits 6-7 en PORTB (PB0 y PB1)
}

/*void asciiVal(void){
    PORTD = buffLast<<2;    // Versi�n anterior: desplaza los bits 2 posiciones
    PORTB = buffLast>>6;    // Desplaza los bits 6 posiciones a la derecha
}*/

void MenuL0(){
    // Muestra el men� de opciones por el puerto serie
    writeTextUART("\nElige una opci�n:\n");
    writeTextUART("1. Leer potenciometro\n");
    writeTextUART("2. Enviar Ascii\n");
}

void ADC_init(void)
{
    // Configuraci�n del voltaje de referencia: AVCC con capacitor externo en AREF
    ADMUX |=  (1<<REFS0);    // REFS0 = 1
    ADMUX &=~ (1<<REFS1);    // REFS1 = 0
    
    ADMUX &=~ (1<<ADLAR);    // Ajusta el resultado a la derecha (ADLAR = 0)

    // Configuraci�n del prescaler para el ADC
    ADCSRA |= (1<<ADPS0);    // ADPS0 = 1
    ADCSRA |= (1<<ADPS1);    // ADPS1 = 1
    ADCSRA |= (1<<ADPS2);    // ADPS2 = 1
                             // Prescaler = 128, frecuencia ADC = 16MHz/128 = 125KHz
    
    ADCSRA |= (1<<ADEN);     // Habilita el ADC (ADEN = 1)
}

uint16_t adcRead(uint8_t canal)
{
    canal &= 0b00000111;            // Limita la entrada a canales 0-7 (la mayor�a de AVR tienen 8 canales ADC)
    ADMUX = (ADMUX & 0xF0) | canal; // Conserva los 4 bits superiores (configuraci�n) y establece los bits de selecci�n de canal
    ADCSRA |= (1<<ADSC);            // Inicia la conversi�n (ADSC = 1)
    while(ADCSRA & (1<<ADSC));      // Espera hasta que se complete la conversi�n (ADSC vuelve a 0)
    return ADC;                     // Devuelve el valor le�do del ADC
}

