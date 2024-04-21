/* SM26CP - Exemplo Portas E/S: verificação de 2 entradas digitais:
 *              -> P1.3 com INTERRUPCAO HABILITADA! - Chave liga o pisca led
 *              -> P1.5 com INTERRUPCAO HABILITADA! - Desligar o pisca led.
 *
 */


#include <msp430.h> 

char on_off = 0;

/**
 * main.c
 */
void main(void)
{
    unsigned long i = 0;


    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    __enable_interrupt(); // Habilita a CPU a aceitar Requisicoes de Interrupcao (IRQ) de qualquer periferico

    /* Inicializacao da PORTA 1
     *  P1.0 - Led Vermelho - saida em nivel baixo
     *  P1.3 - Chave S2 - entrada digital com resistor de pull-up
     *                    com INT. HABILITADA por borda borda de DESCIDA
     *
     *  P1.5 - Chave adicional - entrada digital com resistor de pull-down
     *                    com INT. HABILITADA por borda borda de SUBIDA
     *
     *  P1.1-1.7 - N.C. - saidas em nivel baixo
     *
     *  P1DIR - Especifica se os pinos serao entradas ou saidas digitais
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P1DIR ini.      0  0  0  0      0  0  0  0
     *      P1DIR desejado  1  1  0  1      0  1  1  1
     *                   --------------------------------
     *                  0x       D               7       --->  0xD7
     *                  Decimal -> ??
     *      P1DIR = 0xD7; ou
     *      P1DIR = 0b11010111;
     *      P1DIR = BIT0 + BIT1 + BIT2 + BIT4 + BIT6 + BIT7;
     *      P1DIR = BIT0 | BIT1 | BIT2 | BIT4 | BIT6 | BIT7;
     *      P1DIR = ~(BIT3 + BIT5);
     *
     *
     *  P1REN - Conecta o resistor interno ao pino
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P1REN ini.      0  0  0  0      0  0  0  0
     *      P1REN desejado  0  0  1  0      1  0  0  0
     *                   --------------------------------
     *                  0x       2               8       --->  0x08
     *                  Decimal -> 40
     *      P1REN = 0x28; ou
     *      P1REN = 0b00101000;
     *      P1REN = BIT3 + BIT5;
     *
     *
     *  P1OUT - Define os niveis das saidas digitais
     *
     *                             ---> Define a funcao do resistor P1.5: 0 - pull-down
     *                            |
     *                            |          ---> Define a funcao do resistor P1.3: 1 - pull-up
     *                            |         |
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
     * P1IES - Seleciona a borda de INT: 0 - borda de subida / 1 - borda de descida
     *
     *
     *                             ---> Configura deteccao de borda de int. P1.5: 0 - SUBIDA
     *                            |
     *                            |          ---> Configura deteccao de borda de int. P1.3: 1 - DESCIDA
     *                            |         |
     *      bit             7  6  5  4      3  2  1  0
     *      P1IES ini.      0  0  0  0      0  0  0  0
     *      P1IES desejado  0  0  0  0      1  0  0  0
     *                   --------------------------------
     *                  0x      0               8        --->  0x08
     *                  Decimal ->  8
     *
     *      P1IES = 0x08; ou
     *      P1IES = BIT3;
     *
     * P1IFG - Flags de INT
     *         --> Antes de Habilitar a Int de P1.3, deve-se limpar as flags
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P1IFG ini.      X  X  X  X      X  X  X  X
     *      P1IFG desejado  0  0  0  0      0  0  0  0
     *                   --------------------------------
     *                  0x      0               0        --->  0x00
     *                  Decimal ->  0
     *
     *      P1IFG = 0x00;
     *
     *
     * P1IE - Habilita a INT da Porta 1
     *
     *      bit             7  6  5  4      3  2  1  0
     *      P1IE ini.       0  0  0  0      0  0  0  0
     *      P1IE desejado   0  0  1  0      1  0  0  0
     *                   --------------------------------
     *                  0x      2               8        --->  0x28
     *                  Decimal ->  40
     *
     *      P1IE = 0x28; ou
     *      P1IE = BIT3 + BIT5;
     *
     */
    P1DIR = BIT0 + BIT1 + BIT2 + BIT4 + BIT6 + BIT7;
    P1REN = BIT3 + BIT5;
    P1OUT = BIT3;

    P1IES = BIT3;
    P1IFG = 0x00;
    P1IE = BIT3 + BIT5;

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

        if( on_off ){
            P1OUT ^= BIT0; // Alterna o estado do LED
            for(i=0; i<150000; i++){
                // Nao faz nada
            }
        }else{
            P1OUT &= ~BIT0;
        }


    }while(1);
}


#pragma vector=PORT1_VECTOR
__interrupt  void RTI_da_Porta_1(void){
    // 1 - Precisa ler as flags (P1IFG) para identifica a origem da INT
    //     --> eh necessario limpar a flag setada!

    switch(P1IFG & (BIT3 + BIT5)){
        case BIT3: // Significa que a chave S2 foi pressionada: ligar pisca
            P1IFG &= ~BIT3; // Limpa Flag correspondente a P1.3
            on_off = 1;
            break;
        case BIT5: // Significa que a chave adicional foi pressionada: desligar pisca
            P1IFG &= ~BIT5; // Limpa Flag correspondente a P1.5
            on_off = 0;
            P1OUT &= ~BIT0;
            break;
        case BIT3 + BIT5:// Significa que as chaves S2 e adicional foram pressionadas: nao faz nada
            P1IFG &= ~(BIT3 + BIT5); // Limpa as Flags correspondentes a P1.3 e P1.5
        break;
    }

}






