/*
    DCA/ CT / UFRN 
    Avaliação 1ª unidade - Sistemas Digitais 
    Alunos: Alisson Sousa Moreira e  Maria Alice de Melo Sousa
    2021.2
    Sistema embarcado em MCU para o controle de um secador de grãos 
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// definição das macros
#define set_bit(y, bit_x) (y |= (1 << bit_x)) // valor do pino em 1
#define clr_bit(y, bit_x) (y &= ~(1 << bit_x)) //  o valor do pino em 0
#define cpl_bit(y, bit_x) (y ^= (1 << bit_x)) // trocar estado lógico de um pino
#define tst_bit(y, bit_x) (y & (1 << bit_x)) // lê o valor de um pino

#define LED PB5 
#define conv_ascii 48 // usado para converter em ascii os caracteres recebidos na comunicação serial
#define tam_vetor 5 // constante com o tamanho para vetores


volatile uint8_t cont = 0, seg = 1,  tmp1, tmp2, estado = 0, aux = 0; // variáveis de controle de tempo e da curva.

volatile uint16_t sensor1, sensor2; // variáveis dos sensores. Podemos economizar memória usando inteiros de 16 bits para
				    // armazenar os valores.



uint16_t readSensor(uint8_t sensor){
  
  // configura o AD no canal 0 mantendo AVCC como referência
  ADMUX &= 0b01000000;

  if(sensor != 0){
    ADMUX |= 0b00000001; // se for diferente do canal zero, define como canal 1
  }
  
  ADCSRA |= 0b01000000; // inicie a conversão
  while (!(ADCSRA & 0b00010000)); // espere a conversão ser finalizada (ADIF = 1)

  return ADC;

}

// interrupção por estouro do timer2 
ISR(TIMER2_OVF_vect) { 
  
  sensor1 = readSensor(0); // leia o sensor no canal 0
  if (sensor1 == 0) sensor1 = 1; // verifica se é zero o valor. Se for, coloca em 1 para não gerar erros de cálculo
	OCR1A = (sensor1>>2); // divide por 4 o valor lido (0 - 1023) para ficar dentro do range do pwm (0 - 255)
  sensor2 = readSensor(1); // leia o sensor no canal 1
	if (sensor2 == 0) sensor2 = 1; // verifica se é zero o valor. Se for, coloca em 1 para não gerar erros de cálculo
  OCR1B = (sensor2>>2); // divide por 4 o valor lido (0 - 1023) para ficar dentro do range do pwm (0 - 255)
  
	// verifica se botão de iniciar foi pressionado. se for, altera a variável estado e aux para
	// iniciar o processo de secagem e não permitir que durante a execução, o botão seja pressionado

	if ( !tst_bit(PINB, 0) ) {

    if (estado == 0 && aux == 0) {
      estado = 1;
      aux = 1;
    } else {
      if (aux == 1 && seg > 3) {
        aux = 0;
        estado = 0;
      }
    }
  }
	
	if (estado) { 
		
		cont++; // a cada 16.384 ms, conta 1 para controlar o tempo.
		if (cont == 61) {
			seg++; // 61 estouros do timer é aproximadamente 1 segundo.
			cont = 0;
		}

		// início do cálculo da curva.
		if (seg <= 10) {
      OCR0A = 0;
      OCR0A = (int) (2.55 * ( ((4 * seg) + (2.8 / sensor1 )) * (1.2 / 1)));
      OCR0B = OCR0A;
      //tmp1 = OCR0A/2.55;
      tmp1 = OCR0A;
    } else {
      if (seg < 20) {
        OCR0A = (int) (tmp1);
        OCR0B = OCR0A;
        tmp2 =  OCR0A;
      } else {
        if (seg < 30) { // Funcao de secagem: PWM = (P(t) + a/sensor1) * b/sensor2
          OCR0A = (int) (2.55 * (tmp2 + ( (4 * (seg - 25)) + (2.8 / sensor1)) * (1.2 / 1)));// 3.3 valor de a 1.2 valor de b
          OCR0B = OCR0A;
          //tmp1 = OCR0A/2.55;
          tmp1 = OCR0A;
        } else {
          if (seg < 40) {
            OCR0A = (int) (tmp1);
            OCR0B = OCR0A;
            tmp2 = tmp1;
          } else {
            if (seg < 60) {
              OCR0A = (int) (2.55 * (tmp2 - ( (8 * (seg - 50)) + (2.8 / sensor1)) * (1.2 / 1) ));
              OCR0B = OCR0A;
            } else {
              seg = 1;
              OCR0A = 0;
              OCR0B = 0;
              estado = 0;
              aux = 0;
						}
					}
				}
			}
		}
	}
}




int main() {

  // Configura o PORTD inteiro como saída
  DDRD = 0b11111111;
  
  // Configura os pinos 1, 2 e 3 do PORTB como saídas
  DDRB |= 0b00001110;
  PORTB |= 0xFF; // alta impedância nos pinos do PORTB que são entradas e nível alto nos pinos que estão como saída

  
  ADMUX = 0b01000000; 
  ADCSRA = 0b10000111; 

  // Configurando PWM usando o TIMER0
  TCCR0A = 0b10100011; 
  TCCR0B = 0b00000011; 
  OCR0A = 0;    
	OCR0B = 0;
	
  // CONFIGURAÇÃO DO PWM TIMER1
  TCCR1A = 0b10100010;    
  TCCR1B = 0b00011001;    
  ICR1 = 255;    //valor máximo para contagem

  // CONFIGURANDO A INTERRUPÇÃO DO TIMER2
  cli(); // desliga interrupções globais
  TCCR2B = 0b00000111; 
  TIMSK2 = 0b00000001; 
  sei(); 

}


//------------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp)
{
  unsigned char n;
  for (n = 0; n < tam_vetor; n++)
    disp[n] = 0 + conv_ascii;
  do
  {
    *disp = (valor % 10) + conv_ascii;
    valor /= 10;
    disp++;
    //limpa vetor para armazenamento dos digitos
    //pega o resto da divisão por 10
    //pega o inteiro da divisão por 10
  } while (valor != 0);
}
//-----------------------------------------------------------------------------------

//Usamos essa função apenas para percorrer um vetor de char (string)
//e escrevê-la na saída serial.
void escreve_USART(char *c)
//escreve String
{
  for (; *c != 0; c++) txByte(*c);
}

void escreve_USART(unsigned char c[], int t){
  for(int i=t; i>=0; i--){
    txByte(c[i]);
  }  
}