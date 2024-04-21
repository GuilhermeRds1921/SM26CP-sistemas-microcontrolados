/* SM26CP - Exemplo Portas E/S: verificação de uma entrada digital
 *          Objetivo: Usar a chave S2 para ligar/desligar o pisca led.
 *
 */


#include <msp430.h> 


/**
 * main.c
 */
void main(void)
{
    unsigned long i = 0;


    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    /* Inicializacao da PORTA 1
     *  P1.0 - Led Vermelho - saida em nivel baixo
     *  P1.3 - Chave S2 - entrada digital com resistor de pull-up
     *  P1.1-1.7 - N.C. - saidas em nivel baixo
     *
     *  P1DIR - Especifica se os pinos serao entradas ou saidas digitais
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P1DIR ini.      0  0  0  0      0  0  0  0
     *      P1DIR desejado  1  1  1  1      0  1  1  1
     *                   --------------------------------
     *                  0x       F               7       --->  0xF7
     *                  Decimal -> ??
     *      P1DIR = 0xF7; ou
     *      P1DIR = 0b11110111;
     *      P1DIR = BIT0 + BIT1 + BIT2 + BIT4 + BIT5 + BIT6 + BIT7;
     *      P1DIR = BIT0 | BIT1 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7;
     *      P1DIR = ~BIT3;
     *
     *
     *  P1REN - Conecta o resistor interno ao pino
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P1REN ini.      0  0  0  0      0  0  0  0
     *      P1REN desejado  0  0  0  0      1  0  0  0
     *                   --------------------------------
     *                  0x       0               8       --->  0x08
     *                  Decimal -> 8
     *      P1REN = 0x08; ou
     *      P1REN = 0b00001000;
     *      P1REN = BIT3;
     *
     *
     *  P1OUT - Define os niveis das saidas digitais
     *
     *                                       ---> Define a funcao do resistor: 0 - pull-down / 1 - pull-up
     *                                      |
     *      bit             7  6  5  4      3  2  1  0
     *      P1OUT ini.      0  0  0  0      0  0  0  0
     *      P1OUT desejado  0  0  0  0      1  0  0  0
     *                   --------------------------------
     *                  0x      0               8        --->  0x08
     *                  Decimal ->  8
     *
     *      P1OUT = 0x08; ou
     *      P1OUT = 8;
     *      P1OUT = BIT3;
     *      P1OUT = 0b00000000;
     *      P1OUT = ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);
     *
     */
    P1DIR = 0xF7;
    P1REN = BIT3;
    P1OUT = BIT3;

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


        /* Leitura de P1.3 - chave S2
         *
         *      bit        7  6  5  4    3  2  1  0
         *     ~P1IN      /X /X /X /X   /Y /X /X /X
         *      BIT3       0  0  0  0    1  0  0  0
         *              ------------------------------ AND bit-a-bit
         *                 0  0  0  0   /Y  0  0  0   ---> if(  )
         *
         *      if( (~P1IN) & BIT3 ){
         *         // Led pisca
         *      }else{
         *        // Led apagado
         *      }
         *
         */

        if( (~P1IN) & BIT3 ){
            for(i=0; i<150000; i++){
                // Nao faz nada
            }
            P1OUT ^= BIT0; // Alterna o estado do LED
        }else{
            P1OUT &= ~BIT0; // Apaga o LED
        }


    }while(1);
}
