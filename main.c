/*
 * File:   main.c
 * Author: claudio.paula
 *
 * Created on 29 de Julho de 2019, 13:17
 */


// 'C' source line config statements
 
// CONFIG
#pragma config FOSC = INTRC_NOCLKOUT
//#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
 
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
 
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
 
#define _XTAL_FREQ 4000000
 
void I2C_init (uint32_t clock)
{
    SSPADD = (_XTAL_FREQ/(4*clock))-1;  // here clock is the BR/clock speed    
    SSPCON = 0b00101000;     //first 4 bits I2c master mode , 6th bit is SSPEN enables the scl and sda line
    SSPSTAT = 0;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
}
 
I2C_wait (void)
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));    // wait for start bit to clear in SSPSTAT and bits 0 to 4 in SSPCON2
}
 
void I2C_start (void)
{
    I2C_wait ();
    SSPCON2 |= 0x01; // SEN=1 -> initiate the START condition on SDA and SCL pins
}
 
 
void I2C_repeated_start (void)
{
    I2C_wait();
    SSPCON2 |= 0x02; // RSEN=1  -> initiate REPEATED START condition on SDA and SCL pins
}
 
 
void I2C_stop (void)
{
    I2C_wait ();
    SSPCON2 |= 0x04; // PEN=1 -> initiate the STOP condition on SDA and SCL pins
}
 
 
void I2C_write (uint8_t data)
{
    I2C_wait ();
    SSPBUF = data;  // load data into SSPBUF register
}
 
 /*
uint8_t I2C_read (uint8_t ack)
{
    uint8_t temp;
    I2C_wait();
    RCEN = 1;    // enable receive mode for I2c 
    I2C_wait();
    temp = SSPBUF;   // load data from Buffer to the temp
    I2C_wait();
    ACKDT = (ack);  // 0-- not ACK , 1-- ACK
    ACKEN = 1;   // Send Acknowledgement
    return temp;
}
 */
#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup
 
void lcd_send_data (unsigned char data)
{
	unsigned char data_l, data_u;
	data_l = (data<<4)&0xf0;  //select lower nibble by moving it to the upper nibble position
	data_u = data&0xf0;  //select upper nibble
 
	I2C_start();
	I2C_write (SLAVE_ADDRESS_LCD);
	I2C_write (data_u|0x0D);  //enable=1 and rs =1
	I2C_write (data_u|0x09);  //enable=0 and rs =1
 
	I2C_write (data_l|0x0D);  //enable =1 and rs =1
	I2C_write (data_l|0x09);  //enable=0 and rs =1
 
	I2C_stop();
}
 
void lcd_send_cmd (unsigned char data)
{
	unsigned char data_l, data_u;
	data_l = (data<<4)&0xf0;  //select lower nibble by moving it to the upper nibble position
	data_u = data&0xf0;  //select upper nibble
 
	I2C_start();
	I2C_write (SLAVE_ADDRESS_LCD);
	I2C_write (data_u|0x0C);  //enable=1 and rs =0
	I2C_write (data_u|0x08);  //enable=0 and rs =0
 
	I2C_write (data_l|0x0C);  //enable =1 and rs =0
	I2C_write (data_l|0x08);  //enable=0 and rs =0
 
	I2C_stop();
}
 
void lcd_init (void)
{
    lcd_send_cmd (0x01);
	lcd_send_cmd (0x02);
	lcd_send_cmd (0x28);
	lcd_send_cmd (0x0c);
	lcd_send_cmd (0x80);
}
 
void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
 
//void lcd_send_float (float data, int numberofdigits)  
//{
//	char xg_char[12];
//	snprintf (xg_char, 8, "%f", data);
//	for (int i=0; i<numberofdigits; i++)
//	{
//		lcd_send_data (xg_char[i]);
//	}
//}
 
 
 
void main (void)
{
    
    I2C_init (100000);
    lcd_init ();
   //  lcd_send_cmd(0x01);
     
    while(1) {
       
    lcd_send_string ("CHOCADEIRA REV.1");
    lcd_send_cmd (0xc0);
    lcd_send_string ("TEMPERATURA 30  ");
    
    }

   
}
