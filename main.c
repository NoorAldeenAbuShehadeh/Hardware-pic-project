/*
 * File:   Project_HW.c
 * Author: Momen & Noor Aldeen
 *
 * Created on December 2, 2022, 8:18 PM
 */


// PIC18F4620 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define _XTAL_FREQ   4000000UL
#include <xc.h>
#include<stdio.h>
#include"LCD_Name.h"
#include "my_adc.h"

void initPorts(void)
{
    ADCON1 = 0xF; // or ox0C means 3 analog inputs
    LATA = LATB = LATC = LATD = LATE =0;
    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISD = TRISE =0;
    TRISC = 0b10000000;//0x80
    TRISE = 0x07;
    
}
void movmentClose(void) // 0 degree
{
    PORTCbits.RC2=1;
        __delay_us(1000);
        PORTCbits.RC2=0;
        __delay_us(19000);
        delay_ms(100);
    return;
}
void movmentOpen(void) //90 degree
{
    PORTCbits.RC2=1;
        __delay_us(2000);
        PORTCbits.RC2=0;
        __delay_us(18000);
        delay_ms(100);
    return;
}
float AN[6]={0,0,0,0,0,0};
float SP=2.5;
char buffer1 [16];
char buffer2 [16];
int available [4]={1,1,1,1};
int count =4; 
int doorStatus=0;

void main(void) {
    INTCON = 0; 
    initPorts();
    lcd_init();
    init_adc_no_lib();
    lcd_putc('\f'); //clears the display
    movmentClose();
    PORTCbits.RC2=0;
    float prev5=0.0;
    float prev6=0.0;
    int counttclose=0,counttoclosel=0;
    while(1){ 
        CLRWDT();

//        movmentClose();
//        delay_ms(5000);
//        movmentOpen();
//        delay_ms(5000);
        count=4;
        prev5=AN[4];
        prev6=AN[5];
        for (int i=0; i<6;i++)
        {
          AN[i]=read_adc_voltage((unsigned char) i);
          if((AN[i] <=SP) && i<4)
            {
                count --;
                available[i]=0;
            }
          else
            {
              available[i]=1;
            }
        }
        
        if (AN[4] <=SP)
        {
            if(count == 0)
            {
                if(doorStatus)
                    movmentClose();
            }
            else{
                movmentOpen();
                doorStatus=1;
                counttclose=0;
//                delay_ms(2000);
//                movmentClose();
            }
        }
        else if(AN[5] <=SP)
        {
            movmentOpen();
            doorStatus=1;
            counttclose=0;
//            delay_ms(2000);
//            movmentClose();
        }
        if(prev5<=SP&&AN[4]>SP){
            counttclose++;
        }
        if(prev6<=SP&&AN[5]>SP){
            counttclose++;
        }
        if(counttclose>0){
            delay_ms(1);
            counttclose++;
        }
        if(counttclose==80){
            movmentClose();
            movmentClose();
            doorStatus=0;
            counttclose=0;
        }


        if(count>0) sprintf(buffer1,"AP : %s%s%s%s",available[0]?"P1 ":"",available[1]?"P2 ":"",available[2]?"P3 ":"",available[3]?"P4":"");
        else sprintf(buffer1,"AP : FULL");
        sprintf(buffer2,"Count AP : %d",count);
        lcd_gotoxy(1, 1);
        lcd_puts("                ");
        lcd_gotoxy(1, 1);
        lcd_puts(buffer1);
        lcd_gotoxy(1, 2);
        lcd_puts(buffer2);
    }
    return;
}
