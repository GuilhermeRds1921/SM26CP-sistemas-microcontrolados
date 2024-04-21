#include <msp430.h> 
#define tVermelho 1000000
#define tAmarelo 200000
#define tVerde 800000
#define P1VERDE BIT4
#define P1AMARELO BIT5
#define P1VERMELHO BIT6
#define P2VERDE BIT0
#define P2AMARELO BIT1
#define P2VERMELHO BIT2


/**
 * main.c
 */
void ini_Porta_1(void);
void ini_Porta_2(void);

unsigned char estado = 0;
long i =0;


void main(void)
{

    WDTCTL = WDTPW | WDTHOLD;
    ini_Porta_1();
    ini_Porta_2();
    __enable_interrupt();
    do{
         P1OUT |= P1VERDE;
         P2OUT |= P2VERMELHO;

         for (i =0; i < tVerde; i++){
             if(estado==1){
                 i = tVermelho;
             }
         }
         P1OUT &= ~P1VERDE;
         P1OUT |= P1AMARELO;

         for (i =0; i < tAmarelo; i++);
         P1OUT &= ~P1AMARELO;
         P1OUT |= P1VERMELHO;

         P2OUT &= ~P2VERMELHO;
         P2OUT |= P2VERDE;

         for (i =0; i < tVermelho - tAmarelo; i++){
             if(estado==1){
                    i = tVermelho;
             }
         }

         P2OUT &= ~P2VERDE + BIT5;
         P2OUT |= P2AMARELO;

         for (i =0; i < tAmarelo; i++);

         P2OUT &= ~P2AMARELO;
         P1OUT &= ~P1VERMELHO;

         estado = 0;
    }while(1);

}

#pragma vector=PORT1_VECTOR
__interrupt void RTI_da_Porta_1(void){
    P1IFG &= ~BIT3;
    if(estado == 1){
        estado = 0;
    }else{
        estado = 1;
    }
}

void ini_Porta_1 (){
    P1DIR = ~BIT3;
      P1REN = 0x08;
      P1OUT = 0x08;
      P1IES = 0x08;
      P1IFG = 0;
      P1IE = 0x08;

}

void ini_Porta_2 (){
    P2SEL = 0;
    P2SEL2 = 0;
    P2DIR = 0xFF;
    P2OUT = 0;
}


