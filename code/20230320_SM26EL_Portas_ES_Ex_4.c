/* EXEMPLO 3 - PORTAS ES: configurando 2 entradas com INTERRUPCAO
 *      --> Entrada - P1.3 - Chave S2 + INTERRUPCAO
 *          * Liga o pisca led
 *      --> Entrada - P1.5 - Chave adicional + INTERRUPCAO
 *          * DESLiga o pisca led
 *
 *
 */


#include <msp430.h> 

unsigned char on_off = 0;

/**
 * main.c
 */
void main(void)
{
    long i = 0;

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    __enable_interrupt(); // Habilita a CPU a aceitar Requisicoes de Interrupcao (IRQ)

    /* CONFIG. da PORTA 1
     *
     * P1.0 - LED VM - Saida em nivel baixo
     * P1.3 - Chave S2 (conectada em GND) - entrada digital com INTERRUPCAO HABILITADA por BORDA DE DESCIDA
     * P1.5 - Chave adicional (conectada no Vdd) - entrada digital com INTERRUPCAO HABILITADA por BORDA DE SUBIDA
     * P1.x - N.C. - demais como saida em nivel baixo
     *
     * P1DIR - DIRecao de sinal do pino (entrada/saida)
     *
     *      bit             7  6  5  4    3  2  1  0
     *      P1DIR ini       0  0  0  0    0  0  0  0
     *      P1DIR desejado  1  1  0  1    0  1  1  1
     *                    ---------------------------
     *                    0x    D              7
     *
     *      P1DIR = 0xD7; ou P1DIR = 0b11010111;
     *      ou P1DIR = BIT0 + BIT1 + BIT2 + BIT4 + BIT6 + BIT7;
     *      ou P1DIR = ~(BIT3 + BIT5);
     *
     *
     * P1REN - Conecta o resistor no pino
     *
     *      bit             7  6  5  4    3  2  1  0
     *      P1REN ini       0  0  0  0    0  0  0  0
     *      P1REN desejado  0  0  1  0    1  0  0  0
     *                   ---------------------------------
     *                   0x      2            8
     *
     *     P1REN = 0x28; ou P1REN = 40; ou P1REN = BIT3 + BIT5;
     *
     *
     * P1OUT - Define o NIVEL das SAIDAS da Porta 1
     *
     *                             ---------> Define a funcao do Resistor de P1.5: Pull-Down
     *                            |
     *                            |        -> Define a funcao do Resistor de P1.3: Pull-Up
     *                            |       |
     *      bit             7  6  5  4    3  2  1  0
     *      P1OUT ini       0  0  0  0    0  0  0  0
     *      P1OUT desejado  0  0  0  0    1  0  0  0
     *                   ---------------------------------
     *                   0x      0            8
     *
     *      P1OUT = 0x08; ou P1OUT = 8; P1OUT = 0b00001000;
     *      ou P1OUT = BIT3;
     *
     *  P1IES - Seleciona a borda de interrupcao: 0 - subida/ 1 -descida
     *
     *                             ---------> Borda de interrupcao: 0 - SUBIDA
     *                            |
     *                            |        -> Borda de interrupcao: 1 - DESCIDA
     *                            |       |
     *      bit             7  6  5  4    3  2  1  0
     *      P1IES ini       0  0  0  0    0  0  0  0
     *      P1IES desejado  0  0  0  0    1  0  0  0
     *                   ---------------------------------
     *                   0x      0            8
     *
     *      P1IES = 0x08; ou P1IES = 8; P1IES = 0b00001000;
     *      ou P1IES = BIT3;
     *
     *  P1IFG - Flags de Interrupcao da Porta 1
     *
     *      bit             7  6  5  4    3  2  1  0
     *      P1IFG ini       X  X  X  X    X  X  X  X
     *      P1IFG desejado  0  0  0  0    0  0  0  0
     *                   ---------------------------------
     *                   0x      0             0
     *
     *      P1IFG = 0x00; ou P1IFG &= ~BIT3;
     *
     *
     *  P1IE - HABILITA interrupcao:
     *
     *      bit            7  6  5  4    3  2  1  0
     *      P1IE ini       0  0  0  0    0  0  0  0
     *      P1IE desejado  0  0  1  0    1  0  0  0
     *                   ---------------------------------
     *                    0x      2            8
     *
     *      P1IE = 0x28; ou P1IE = 40; P1IE = 0b00101000;
     *      ou P1IE = BIT3 + BIT5;
     *
     *
     */
    P1DIR = BIT0 + BIT1 + BIT2 + BIT4 + BIT6 + BIT7;
    P1REN = BIT3 + BIT5;
    P1OUT = BIT3;

    P1IES = BIT3;
    P1IFG = 0x00; // Antes de habilitar a INT eh necessario limpar Flags de Int.
    P1IE = BIT3 + BIT5;

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

        if(on_off){ // pisca ligado
            for(i=0;i<200000;i++){
               // vazio
            }
            P1OUT ^= BIT0;
        }else{  // Chave nao-pressionada
            // Apaga Led
            P1OUT &= ~BIT0;
        }



    }while(1);
}

// RTI da PORTA 1
#pragma vector=PORT1_VECTOR
__interrupt void RTI_da_Porta_1(void){

    // 1 - Identificar qual entrada provocou a INT e LIMPAR a flag de int.
    /*
     *
     *      bit             7  6  5  4    3  2  1  0
     *      P1IFG           X  X  Z  X    Y  X  X  X
     *      BIT3 + BIT5     0  0  1  0    1  0  0  0
     *                   --------------------------------- AND b-a-b
     *                      0  0  Z  0    Y  0  0  0
     *
     *      Se S2 for pressionada              --> 0  0  0  0    1  0  0  0  -> 0x08 ou = BIT3
     *      Se Chave adicional for pressionada --> 0  0  1  0    0  0  0  0  -> 0X20 ou = BIT5
     *      Se AMBAS forem pressionadas        --> 0  0  1  0    1  0  0  0  -> 0x28 OU = BIT3 + BIT5
     *
     *      switch( P1IFG & (BIT3 + BIT5) ){
     *          case BIT3:
     *             - Significa que S2 foi pressionada
     *                  --> Limpa a flag ref. ao BIT3 de P1IFG
     *                  --> Realiza a acao da chave S2: Ligar Pisca Led
     *          break;
     *          case BIT5:
     *             - Significa que ADICIONAL foi pressionada
     *                  --> Limpa a flag ref. ao BIT5 de P1IFG
     *                  --> Realiza a acao da chave ADICIONAL: Desligar Pisca Led
     *          break;
     *          case (BIT3 + BIT5):
     *             - Significa que AMBAS as chaves foram pressionadas
     *                  --> Limpa a flag ref. ao BIT3 e BIT5 de P1IFG
     *                  --> Qual acao a ser realizada: IGNORA/Nao faz nada
     *          break;
     *      }
     *
     */

     switch( P1IFG & (BIT3 + BIT5) ){
         case BIT3:
            // Significa que S2 foi pressionada
            P1IFG &= ~BIT3;     // --> Limpa a flag ref. ao BIT3 de P1IFG
            on_off = 1;         // --> Realiza a acao da chave S2: Ligar Pisca Led
         break;
         case BIT5:
            // - Significa que ADICIONAL foi pressionada
            P1IFG &= ~BIT5;    // --> Limpa a flag ref. ao BIT5 de P1IFG
            on_off = 0;        // --> Realiza a acao da chave ADICIONAL: Desligar Pisca Led
            P1OUT &= ~BIT0;    // Apaga o Led
            break;
         case (BIT3 + BIT5):
            // - Significa que AMBAS as chaves foram pressionadas
            P1IFG &= ~(BIT3 + BIT5);     // --> Limpa a flag ref. ao BIT3 e BIT5 de P1IFG
                 // --> Qual acao a ser realizada: IGNORA/Nao faz nada
         break;
     }

}














