//***************************************************************************
// Universidad del Valle de Guatemala
// IE2023: Programación de Microcontroladores 
// Hardware: ATMEGA328PB
// Created: 20/04/2025
// Descripción: Comunicación UART con ATMEGA328PB
//                 Parte 1: Enviar un carácter al PC
//                 Parte 2: Recibir un carácter del PC y mostrarlo en el PORTB
//***************************************************************************
// PostLab 6
//****************************************************************************/

// Define la frecuencia del reloj del microcontrolador (16MHz)
#define F_CPU 16000000UL  

// Inclusión de bibliotecas necesarias
#include <avr/io.h>        // Biblioteca estándar para E/S de AVR
#include <util/delay.h>    // Funciones de retardo
#include <avr/interrupt.h> // Manejo de interrupciones
#include <stdio.h>         // Funciones estándar de E/S (para sprintf)

// Declaración de prototipos de funciones
void initUART (void);           // Inicialización de comunicación UART
void writeUART (char caracter); // Envía un solo carácter por UART
void writeTextUART (char* texto); // Envía una cadena de texto por UART
void asciiVal(void);            // Muestra el valor ASCII en los puertos
void MenuL0(void);              // Muestra el menú principal
void ADC_init(void);            // Inicializa el conversor analógico-digital
uint16_t adcRead(uint8_t);      // Lee un valor del ADC de un canal específico

// Variables globales
int adcValue1 = 0;        // Almacena el valor leído del ADC
uint8_t StateAscii = 0;   // Estado para la función de mostrar caracteres ASCII
uint8_t StatePot = 0;     // Estado para la función de leer potenciómetro
char buffLast = '0';      // Último carácter recibido por UART
char buffer[10];          // Buffer para convertir valores numéricos a texto

// Variable para almacenar caracteres recibidos por interrupción
volatile char bufferRX;   // El modificador 'volatile' indica que esta variable puede cambiar en cualquier momento (por la ISR)

int main(void)
{
    // Inicialización de módulos
    initUART();    // Configura la comunicación UART
    ADC_init();    // Configura el ADC
    
    sei();         // Habilita las interrupciones globales (Set Interrupt Flag)
    DDRD = 0xFF;   // Configura todo el puerto D como salida
    DDRB = 0xFF;   // Configura todo el puerto B como salida
    
    MenuL0();      // Muestra el menú principal por el puerto serie
    PORTD = 0;     // Inicializa el puerto D en bajo
    PORTB = 0;     // Inicializa el puerto B en bajo
    
    // Bucle principal del programa
    while (1){
        asciiVal();    // Actualiza la visualización del valor ASCII en los puertos
        
        // Si está activado el modo de lectura del potenciómetro
        if (StatePot == 1){
            adcValue1 = adcRead(5);  // Lee el valor del potenciómetro en el canal 5
            
            // Convierte el valor entero del ADC en una cadena de caracteres
            sprintf(buffer, "%d", adcValue1);
            
            // Envía el valor convertido por el puerto serie
            writeTextUART(buffer);
            
            // Envía un salto de línea
            writeTextUART("\n");
            
            // Espera 1 segundo antes de la siguiente lectura
            _delay_ms(1000);
        }
    }    
}

void initUART(void){
    // Configuración de pines Rx (PD0) y Tx (PD1)
    DDRD &= ~(1<<DDD0);  // Configura PD0 (RX) como entrada
    DDRD |= (1<<DDD1);   // Configura PD1 (TX) como salida
    
    // Configuración del registro UCSR0A
    UCSR0A = 0;                // Limpia el registro
    UCSR0A |= (1<<U2X0);       // Activa el modo de velocidad doble (Fast mode)
    
    // Configuración del registro UCSR0B
    UCSR0B = 0;                // Limpia el registro
    UCSR0B |= (1<<RXCIE0);     // Habilita la interrupción de recepción
    UCSR0B |= (1<<RXEN0);      // Habilita la recepción
    UCSR0B |= (1<<TXEN0);      // Habilita la transmisión
    
    // Configuración del formato de frame: 8 bits de datos, sin paridad, 1 bit de parada
    UCSR0C = 0;                // Limpia el registro
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);  // Configura 8 bits de datos
    
    // Configuración del baudrate a 9600 bps (con U2X0=1)
    UBRR0 = 207;               // Valor calculado para 9600 bps con U2X0=1 y F_CPU=16MHz
}

void writeUART(char caracter){
    // Espera hasta que el buffer de transmisión esté vacío
    while(!(UCSR0A & (1<<UDRE0)));
    // Coloca el carácter en el registro de datos para transmitir
    UDR0 = caracter;
}
    
void writeTextUART(char* texto){
    uint8_t i;
    // Recorre cada carácter hasta encontrar el terminador '\0'
    for (i=0; texto[i] != '\0'; i++){
        // Espera a que el buffer de transmisión esté vacío
        while(!(UCSR0A&(1<<UDRE0)));
        // Envía el carácter actual
        UDR0 = texto[i];
    }
}

// Rutina de servicio de interrupción para recepción UART
ISR(USART0_RX_vect){
    bufferRX = UDR0;  // Lee el carácter recibido del registro de datos
    
    // Si estamos en el modo ASCII (esperando un carácter)
    if(StateAscii >= 1) {
        PORTB |= (1<<PB5);  // Enciende el LED en PB5 para indicar recepción
        buffLast = bufferRX; // Guarda el carácter recibido para mostrarlo en los puertos
    }
    
    // Eco: devuelve el carácter recibido por el puerto serie
    while(!(UCSR0A&(1<<UDRE0)));
    UDR0 = bufferRX;
    
    // Manejo del estado ASCII
    if (StateAscii >= 1){
        StateAscii++;  // Incrementa el estado
        if (StateAscii >= 2){  // Si ya recibimos un carácter
            StateAscii = 0;    // Volvemos al menú principal
            MenuL0();          // Muestra el menú
        }
    }
    
    // Si estamos en modo lectura de potenciómetro y el usuario presiona '0'
    if (StatePot == 1){
        if (bufferRX == '0'){
            StatePot = 0;      // Desactiva el modo potenciómetro
            MenuL0();          // Vuelve al menú principal
        }
    }
    
    // Manejo de las opciones del menú
    if (bufferRX == '1'){  // Opción 1: Leer potenciómetro
        if (StateAscii == 0){  // Si estamos en el menú principal
            writeTextUART("\nValor Potenciometro\n");
            writeTextUART("Escribir 0 para volver\n");
            StatePot = 1;      // Activa el modo potenciómetro
        }
    } else if (bufferRX == '2'){  // Opción 2: Modo ASCII
        if (StateAscii == 0){  // Si estamos en el menú principal
            writeTextUART("\nAscci\n");
            writeTextUART("\nEscribe tu código Ascii:\n");
            StateAscii = 1;    // Activa el modo ASCII
            PORTD = 0;         // Limpia el puerto D
            PORTB = 0;         // Limpia el puerto B
        }
    }
}

void asciiVal(void) {
    // Asegura que buffLast esté en el rango de 0 a 255
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
    PORTD = buffLast<<2;    // Versión anterior: desplaza los bits 2 posiciones
    PORTB = buffLast>>6;    // Desplaza los bits 6 posiciones a la derecha
}*/

void MenuL0(){
    // Muestra el menú de opciones por el puerto serie
    writeTextUART("\nElige una opción:\n");
    writeTextUART("1. Leer potenciometro\n");
    writeTextUART("2. Enviar Ascii\n");
}

void ADC_init(void)
{
    // Configuración del voltaje de referencia: AVCC con capacitor externo en AREF
    ADMUX |=  (1<<REFS0);    // REFS0 = 1
    ADMUX &=~ (1<<REFS1);    // REFS1 = 0
    
    ADMUX &=~ (1<<ADLAR);    // Ajusta el resultado a la derecha (ADLAR = 0)

    // Configuración del prescaler para el ADC
    ADCSRA |= (1<<ADPS0);    // ADPS0 = 1
    ADCSRA |= (1<<ADPS1);    // ADPS1 = 1
    ADCSRA |= (1<<ADPS2);    // ADPS2 = 1
                             // Prescaler = 128, frecuencia ADC = 16MHz/128 = 125KHz
    
    ADCSRA |= (1<<ADEN);     // Habilita el ADC (ADEN = 1)
}

uint16_t adcRead(uint8_t canal)
{
    canal &= 0b00000111;            // Limita la entrada a canales 0-7 (la mayoría de AVR tienen 8 canales ADC)
    ADMUX = (ADMUX & 0xF0) | canal; // Conserva los 4 bits superiores (configuración) y establece los bits de selección de canal
    ADCSRA |= (1<<ADSC);            // Inicia la conversión (ADSC = 1)
    while(ADCSRA & (1<<ADSC));      // Espera hasta que se complete la conversión (ADSC vuelve a 0)
    return ADC;                     // Devuelve el valor leído del ADC
}

