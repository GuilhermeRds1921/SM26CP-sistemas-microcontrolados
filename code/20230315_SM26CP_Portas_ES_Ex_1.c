#include <msp430.h> 


/**
 * main.c
 */
void main(void)
{
    unsigned long i = 0;


	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	/* Inicializacao da PORTA 1
	 *  P1.0 - Led Vermelho - saida em nivel baixo
	 *  P1.1-1.7 - N.C. - saidas em nivel baixo
	 *
	 *  P1DIR - Especifica se os pinos serao entradas ou saidas digitais
	 *
	 *      bit             7  6  5  4      3  2  1  0
	 *      P1DIR ini.      0  0  0  0      0  0  0  0
	 *      P1DIR desejado  1  1  1  1      1  1  1  1
	 *                   --------------------------------
	 *                  0x       F               F       --->  0xFF
	 *                  Decimal -> 255
	 *      P1DIR = 0xFF; ou
	 *      P1DIR = 255; ou
	 *      P1DIR = 0b11111111;
	 *      P1DIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
	 *      P1DIR = BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
	 *
	 *  P1OUT - Define os niveis das saidas digitais
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P1OUT ini.      0  0  0  0      0  0  0  0
     *      P1OUT desejado  0  0  0  0      0  0  0  0
     *                   --------------------------------
     *                  0x      0               0        --->  0x00
     *                  Decimal ->  0
     *
     *      P1OUT = 0x00; ou
     *      P1OUT = 0;
     *      P1OUT = 0b00000000;
     *      P1OUT = ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);
	 *
	 */
	P1DIR = 0xFF;
	P1OUT = 0x00;

    /* Inicializacao da PORTA 2
     *  P2.0-2.7 - N.C. - saidas em nivel baixo
     *
     *  -> MUDAR as FUNCOES dos pinos 18 e 19 de XIN e XOUT para P2.6  e P2.7
     *
     *  P2SEL - Muda a funcao digital dos pinos da Porta 2
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P2SEL ini.      1  1  0  0      0  0  0  0
     *      P2SEL desejado  0  0  0  0      0  0  0  0
     *                  ------------------------------------
     *                  0x       0              0
     *
     *      P2SEL = 0x00; ou P2SEL = 0; ou
     *      P2SEL &= ~(BIT6 + BIT7);
     *
     *  P2DIR - Especifica se os pinos serao entradas ou saidas digitais
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P2DIR ini.      0  0  0  0      0  0  0  0
     *      P2DIR desejado  1  1  1  1      1  1  1  1
     *                   --------------------------------
     *                  0x       F               F       --->  0xFF
     *                  Decimal -> 255
     *      P2DIR = 0xFF; ou
     *      P2DIR = 255; ou
     *      P2DIR = 0b11111111;
     *      P2DIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
     *      P2DIR = BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
     *
     *  P2OUT - Define os niveis das saidas digitais
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P2OUT ini.      0  0  0  0      0  0  0  0
     *      P2OUT desejado  0  0  0  0      0  0  0  0
     *                   --------------------------------
     *                  0x      0               0        --->  0x00
     *                  Decimal ->  0
     *
     *      P2OUT = 0x00; ou
     *      P2OUT = 0;
     *      P2OUT = 0b00000000;
     *      P2OUT = ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);
     *
     */
	P2SEL &= ~(BIT6 + BIT7);
	P2DIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
	P2OUT = 0x00;

	do{
	    // Loop Infinito -> Rotina para o LED piscar
	    for(i=0; i<150000; i++){
	        // Nao faz nada
	    }
	    P1OUT ^= BIT0; // Alterna o estado do LED

	}while(1);
}
