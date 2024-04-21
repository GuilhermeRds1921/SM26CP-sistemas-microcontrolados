#include <msp430.h> 


/**
 * main.c
 */
void main(void)
{
    long i = 0;

	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	/* CONFIG. da PORTA 1
	 *
	 * P1DIR - DIRecao de sinal do pino (entrada/saida)
	 *
	 *      bit             7  6  5  4    3  2  1  0
	 *      P1DIR ini       0  0  0  0    0  0  0  0
	 *      P1DIR desejado  1  1  1  1    1  1  1  1
	 *                    ---------------------------
	 *                    0x    F              F
	 *
	 *      P1DIR = 0xFF; ou P1DIR = 256; ou P1DIR = 0b11111111;
	 *      ou P1DIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
	 *
	 * P1OUT - Define o NIVEL das SAIDAS da Porta 1
	 *
     *      bit             7  6  5  4    3  2  1  0
     *      P1OUT ini       0  0  0  0    0  0  0  0
     *      P1OUT desejado  0  0  0  0    0  0  0  0
	 *
	 *      P1OUT = 0; ou P1OUT = 0x00; P1OUT = 0b00000000;
	 *
	 */
	P1DIR = 0xFF;
	P1OUT = 0x00;

    /* CONFIG. da PORTA 2
     *
     * P2SEL - Altera a FUNCAO dos pinos. Quais? pinos 18 e 19
     *   Pino 19: XIN -->  P2.6
     *   Pino 18: XOUT-->  P2.7
     *
     *      bit             7  6  5  4    3  2  1  0
     *      P2SEL ini       1  1  0  0    0  0  0  0
     *      P2SEL desejado  0  0  0  0    0  0  0  0
     *
     *      P2SEL = 0; ou
     *      P2SEL = P2SEL & ~(BIT6 + BIT7) ou P2SEL &= ~(BIT6 + BIT7);
     *
     *
     * P2DIR - DIRecao de sinal do pino (entrada/saida)
     *
     *      bit             7  6  5  4    3  2  1  0
     *      P2DIR ini       0  0  0  0    0  0  0  0
     *      P2DIR desejado  1  1  1  1    1  1  1  1
     *                    ---------------------------
     *                    0x    F              F
     *
     *      P2DIR = 0xFF; ou P2DIR = 256; ou P2DIR = 0b11111111;
     *      ou P2DIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
     *
     * P2OUT - Define o NIVEL das SAIDAS da Porta 1
     *
     *      bit             7  6  5  4    3  2  1  0
     *      P2OUT ini       0  0  0  0    0  0  0  0
     *      P2OUT desejado  0  0  0  0    0  0  0  0
     *
     *      P2OUT = 0; ou P2OUT = 0x00; P2OUT = 0b00000000;
     *
     */
	P2SEL &= ~(BIT6 + BIT7);
	P2DIR = 0xFF;
	P2OUT = 0;



	do{
	    // Loop infinito

	    for(i=0;i<100000;i++){
	       // vazio
	    }
	    P1OUT |= BIT0;

        for(i=0;i<100000;i++){
           // vazio
        }
        P1OUT &= ~BIT0;



	}while(1);
}
