// DS18B20 - +-0,5°C
// only serial RS485 version - for remote monitoring
// fuses set for Atmega328P L-C6, H-DD
// modbus implemented by using Max B. mbs38 at github code
//--------------------------------------------------------
// thermostat function, for S3,S4, for range 0-90°C
// write thermostat temperature, 
// write delay between reading values , for range 0-100 sec
// setting of address by 2 bits - not implem
// reading ROM code - 8BYTE - not implemented
//-----------------------------

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
//-------------
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
//-----------------
#include "ds18b20.h"
#include "yaMBSiavr.h"

#define clientAddress 0x0B 			// slave address

//--------------------
#define adr_thermostat1	 	0x02
#define adr_thermostat2	 	0x04		//adr in EEPROM - for MODBUS,word 
#define adr_delay_read		0x06
//-------------------- 

//modbus 
uint8_t back_len0 = 0;
uint8_t slave_add; 
//current temperatures
int CurrentTemp1=0; 
int CurrentTemp2=0,CurrentTemp3=0,CurrentTemp4=0,CurrentTemp5=0,CurrentTemp6=0;

//unsigned char CharBuffer[21];
char CharBuffer[21];

//flags
unsigned char ErrFlag=0;
uint8_t Device1Presence=1,Device2Presence=1,Device3Presence=1,Device4Presence=1,Device5Presence=1,Device6Presence=1;

//user char for LCD
//	static const PROGMEM unsigned char userChar[] =
//	{
//	0x18, 0x18, 0x06, 0x09, 0x08, 0x08, 0x09, 0x06,
//	0x07, 0x08, 0x13, 0x14, 0x14, 0x13, 0x08, 0x07,
//	0x00, 0x10, 0x08, 0x08, 0x08, 0x08, 0x10, 0x00
//	};
//--------for bmp085----------------
//int32_t gAltitudeCorr=0; // altitude correction for relative barometric pressure
//int32_t gPressCorr=0;
//long l;
//double d;
//char printbuff[10];

//------modbus

// ----------global variables ------------------
volatile uint8_t instate = 0;
volatile uint8_t outstate = 0;
volatile uint16_t inputRegisters[4];
volatile uint16_t holdingRegisters[20]= {40,41,42,43,44,45,47,48,0,0,0,0,0,0,0,0,29,28,10,0}; // default values for example
void timer0100us_start(void) {
	TCCR0B|=(1<<CS01); //prescaler 8
	TIMSK0|=(1<<TOIE0);
}

/*
*   Modify the following 3 functions to implement your own pin configurations...
*/
// output bits
void SetOuts(volatile uint8_t in) {
	//PORTD|= (((in & (1<<3))<<4) | ((in & (1<<4))<<1) | ((in & (1<<5))<<1));
	//PORTB|= (((in & (1<<0))<<2) | ((in & (1<<1))) | ((in & (1<<2))>>2));
	in=~in;
	//PORTB&= ~(((in & (1<<0))<<2) | ((in & (1<<1))) | ((in & (1<<2))>>2));
	//PORTD&= ~(((in & (1<<3))<<4) | ((in & (1<<4))<<1) | ((in & (1<<5))<<1));
	
}

// input bits
uint8_t ReadIns(void) {
	uint8_t ins=0x00;
	//ins|=(PINC&((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)));
	//ins|=(((PIND&(1<<4))<<2)|((PIND&(1<<3))<<4));
	return ins;
}

void io_conf(void) { 
	/*
	 for RS485 driver
	*/
	DDRD=0x00;
	PORTD=0x00;
	//	DDRD &= ~(1<<0);
	//	DDRD &= ~(1<<1);
	//	PORTD &= ~(1<<1);
	//	PORTD &= ~(1<<2);
	PORTD|=(1<<0);
			//DDRD |= (1<<2)|(1<<5)|(1<<6)|(1<<7);
	DDRD |= (1<<2);
	
	DDRB |= (1<<PB5);	//PB5 LED blink for test
	PORTB |= (1<<PB5);
	//_delay_ms(300);
	//PORTB &=~(1<<PB5);

}




void modbusGet(void) {
	if (modbusGetBusState() & (1<<ReceiveCompleted))
	{
		switch(rxbuffer[1]) {
			case fcReadCoilStatus: {
				modbusExchangeBits(&outstate,0,8);
			}
			break;
			
			case fcReadInputStatus: {
				volatile uint8_t inps = ReadIns();
				modbusExchangeBits(&inps,0,8);
			}
			break;
			
			case fcReadHoldingRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,19);
			}
			break;
			
			case fcReadInputRegisters: {
				modbusExchangeRegisters(inputRegisters,0,4);
			}
			break;
			
			case fcForceSingleCoil: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetSingleRegister: {
				modbusExchangeRegisters(holdingRegisters,14,19); // only addr 10 and higher
			}
			break;
			
			case fcForceMultipleCoils: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetMultipleRegisters: {
				modbusExchangeRegisters(holdingRegisters,14,19);	// only addr 10 and higher
			}
			break;
			
			default: {
				modbusSendException(ecIllegalFunction);
			}
			break;
		}
	}
}
/////////////////////////////////////////////////////////////////////

int main(void)
{
	//int desatiny,cele,j;
	unsigned char chyba1=0,chyba2=0,chyba3=0,chyba4=0,chyba5=0,chyba6=0;
	unsigned char c_ROM_code;
    int thermostat1,thermostat2,delay_read,j;
	int thermostat1_eeprom,thermostat2_eeprom,delay_read_eeprom; //24dec18hex,28dec1C,6dec6hex
	int interval,therm1=0,therm2=0;	// interval*10ms,for example 1000*10ms = 10sec	   
	
	// general definiton for Atmega328
	//DDRB = 0xFF; //set as output
	//PORTB = 0xFF;
	//DDRC = 0xFF;
	//PORTC = 0xFF;
	//DDRD = 0xFF;
	//PORTD = 0xFF;


	// Port C initialization DS18b20 //
	DDRC &= ~(1 << PC0);   // input
    PORTC &= ~(1 << PC0);   
	DDRC &= ~(1 << PC1);   
    PORTC &= ~(1 << PC1);   
	DDRC &= ~(1 << PC2);   
    PORTC &= ~(1 << PC2);   
	DDRC &= ~(1 << PC3);   
    PORTC &= ~(1 << PC3);
	DDRD &= ~(1 << PD3);   
    PORTD &= ~(1 << PD3);   
	DDRD &= ~(1 << PD4);   
    PORTD &= ~(1 << PD4);  
	
		//DDRB = 0xFF; //set as output

		//DDRD |= (1<<PD5) | (1<<PD6);	// output for rele
		//PORTD |= (1<<PD5) | (1<<PD6); //set log.1
		//PORTD &=~(1<<PD5);		// set log. 0
		//PORTD &=~(1<<PD6);		// set log. 0
		//PORTD |= (1 << PD6) – zápis log. 1 na pin 6 portu D
		//PORTD &= ~(1 << PD6) – zápis log.0 na pin 6 portu D 
	//modbus------------------------------------------------------
	io_conf();	
	modbusSetAddress(clientAddress); // setting client address
	modbusInit();
	//wdt_enable(7);
	timer0100us_start();
	//------------------ for rele
	DDRD |=(1 << PD5);   // output
    //PORTD &= ~(1 << PD5);
	PORTD |= (1<<PD5);
	DDRD |=(1 << PD6);   
    //PORTD &= ~(1 << PD6);
	PORTD |= (1<<PD6);
	//---------------------------------------------------------
	//deny interrupt
	cli(); 
	//Watchdog initialization.
	wdt_reset();
	//wdt_enable(WDTO_8S); // for 5V supply,  At lower supply voltages, the times will increase 
	

	thermostat1_eeprom = eeprom_read_word((uint16_t*)adr_thermostat1);   

	thermostat2_eeprom = eeprom_read_word((uint16_t*)adr_thermostat2);   

	delay_read_eeprom = eeprom_read_word((uint16_t*)adr_delay_read);
	

	if((thermostat1_eeprom>0)&&(thermostat1_eeprom<90))
		{
		holdingRegisters[16] = thermostat1 = thermostat1_eeprom;
		}
		else 
		{ 	thermostat1_eeprom=24;
			eeprom_write_word((uint16_t*)adr_thermostat1,thermostat1_eeprom); // works
		}

	if((thermostat2_eeprom>0)&&(thermostat2_eeprom<90))
		{
		holdingRegisters[17] = thermostat2 = thermostat2_eeprom;
		}
		else 
		{thermostat2_eeprom=28;
		eeprom_write_word((uint16_t*)adr_thermostat2,thermostat2_eeprom); // works
		}

	if((delay_read_eeprom>0)&&(delay_read_eeprom<100))
		{
		holdingRegisters[18] = delay_read = delay_read_eeprom;
		}
		else 
		{delay_read_eeprom=10;
		eeprom_write_word((uint16_t*)adr_delay_read,delay_read_eeprom);
		}

	//------------------------------------------ 
	Device1Presence = ds18b20_reset(&PORTC,PC0);
	if(Device1Presence)
			{chyba1=0;}
		else { chyba1=1;}
	_delay_ms(100);
	//---------------------------------
	Device2Presence = ds18b20_reset(&PORTC,PC1);
	if(Device2Presence)
		{chyba2=0;}
		else { chyba2=1;}
	_delay_ms(100);
	//---------------------------------
	Device3Presence = ds18b20_reset(&PORTC,PC2);
	if(Device3Presence)
			{chyba3=0;}
		else { chyba3=1;}
	_delay_ms(100);
	//---------------------------------
	Device4Presence = ds18b20_reset(&PORTC,PC3);
	if(Device4Presence)
			{chyba4=0;}
		else { chyba4=1;}
	//---------------------------------
	Device5Presence = ds18b20_reset(&PORTD,PD3);
	if(Device5Presence)
			{chyba5=0;}
		else {  chyba5=1;}
	//---------------------------------
	Device6Presence = ds18b20_reset(&PORTD,PD4);
	if(Device6Presence)
			{chyba6=0;}
		else { chyba6=1;}
	_delay_ms(100);

	// ----------print ROM code----------------
	/*
	if(chyba1==0)
		{	
			read_ROM_CODE(&PORTC,PC0);
		}

	
	if(chyba2==0)
		{	
			read_ROM_CODE(&PORTC,PC1);
		}
		
	if(chyba3==0)
		{
			read_ROM_CODE(&PORTC,PC2);
		}
			
	if(chyba4==0)
		{
			read_ROM_CODE(&PORTC,PC3);
		}
	*/
	if(chyba5==0)
		{
			//read_ROM_CODE(&PORTD,PD3);
		}

	
	if(chyba6==0)
		{
			read_ROM_CODE(&PORTD,PD4); 
		}
	
		/*
        * load two userdefined characters from program memory
        * into LCD controller CG RAM location 0 and 1
        */
    //   lcd_command(_BV(LCD_CGRAM));  /* set CG RAM start address 0 */
     //  for(i=0; i<24; i++)
     //  {
     //     lcd_data(pgm_read_byte_near(&userChar[i]));
     //  }
		//for test---------------------------------------
	for (j=0;j<5;j++)
		{PORTB |= (1<<PB5);
		 _delay_ms(300);
		 PORTB &= ~(1<<PB5);
		 _delay_ms(400);
		 }
//------------main loop-------------------------------------
	while(1)
 	{

		wdt_reset(); 
		sei();
	//----------------------------------------------------	
		if(chyba1==0)
		{
		CurrentTemp1 = ds18b20_gettemp(&PORTC,PC0); // decicelsius
		}
	//----------------------------------------------------	
		if(chyba2==0)
		{
		CurrentTemp2 = ds18b20_gettemp(&PORTC,PC1);	//decicelsius
		}
	//----------------------------------------------------	
		if(chyba3==0)
		{
		CurrentTemp3 = ds18b20_gettemp(&PORTC,PC2);	//decicelsius
		}
	//----------------------------------------------------	
		if(chyba4==0)
		{
		CurrentTemp4 = ds18b20_gettemp(&PORTC,PC3);	//decicelsius
		}
	//----------------------------------------------------			
		if(chyba5==0)
		{
		CurrentTemp5 = ds18b20_gettemp(&PORTD,PD3);	//decicelsius
		}
	//----------------------------------------------------			
		if(chyba6==0)
		{
		CurrentTemp6 = ds18b20_gettemp(&PORTD,PD4);	//decicelsius
		}
	//----------------------------------------------------			
	//wdt_reset();
	//modbus-------------------------
		holdingRegisters[0]=CurrentTemp1;
		holdingRegisters[1]=CurrentTemp2;
		holdingRegisters[2]=CurrentTemp3;
		holdingRegisters[3]=CurrentTemp4;
		holdingRegisters[4]=CurrentTemp5;
		holdingRegisters[5]=CurrentTemp6;
			c_ROM_code = ROM_code[0];
			holdingRegisters[6] = c_ROM_code;
			c_ROM_code = ROM_code[1];
			holdingRegisters[7]= c_ROM_code;
			c_ROM_code = ROM_code[2];
			holdingRegisters[8]= c_ROM_code;
			c_ROM_code = ROM_code[3];
			holdingRegisters[9]= c_ROM_code;
			c_ROM_code = ROM_code[4];
			holdingRegisters[10]= c_ROM_code;
			c_ROM_code = ROM_code[5];
			holdingRegisters[11]= c_ROM_code;
			c_ROM_code = ROM_code[6];
			holdingRegisters[12]= c_ROM_code;
			c_ROM_code = ROM_code[7];
			holdingRegisters[13]= c_ROM_code;
		//holdingRegisters[6]= 99; 	//reserved
		//holdingRegisters[6] = c_ROM_code;
		//holdingRegisters[7]= 99;	//reserved

		holdingRegisters[14]=therm1;
		holdingRegisters[15]=therm2;
		//holdingRegisters[8]=DDRD;	//for testing
		//holdingRegisters[9]=PORTD;

		thermostat1 = holdingRegisters[16];	// from master 
		thermostat2 = holdingRegisters[17];
		delay_read = holdingRegisters[18];	// from master in seconds
				//relative_corr = holdingRegisters[13];
				//relative_corr = holdingRegisters[14];
		

	if((delay_read > 1) && (delay_read <200))
		{ interval = delay_read*100;
		//
		if(delay_read_eeprom != delay_read)
			{
			 cli();
			 eeprom_write_word((uint16_t*)adr_delay_read,delay_read);
			 sei();
			 delay_read_eeprom = delay_read;
			}
			else delay_read_eeprom = delay_read;
		}
		else interval = 10;

		// testing range, not ended
	if((thermostat1>0)&&(thermostat1<90))
		{
		if((CurrentTemp3) > (thermostat1*10))
				{PORTD  &= ~(1<<PD5);	//set PD5 
				 therm1=1; 
				}
		if((CurrentTemp3 + 8) < (thermostat1*10))
				{ PORTD |= (1<<PD5);
				  therm1=0;	
				}
		
		if(thermostat1 != thermostat1_eeprom)
			{cli();	
			 eeprom_write_word((uint16_t*)adr_thermostat1,thermostat1); // works
			 sei();
			 thermostat1_eeprom = thermostat1;
			}
		}
		else {thermostat1 = thermostat1_eeprom;
			}
	
	
	if((thermostat2>0)&&(thermostat2<90))
		{
		 if((CurrentTemp4) > (thermostat2*10))
				{PORTD  &= ~(1<<PD6);	//set PD6 
				 therm2=1; 
				}
		 if((CurrentTemp4 + 8) < (thermostat2*10))	 
				{ PORTD |= (1<<PD6);
				  therm2=0;	
				}

		 if(thermostat2 != thermostat2_eeprom)
			{cli();	
			 eeprom_write_word((uint16_t*)adr_thermostat2,thermostat2); // works
			 sei();
			 thermostat2_eeprom = thermostat2;
			}
		}
		else {thermostat2 = thermostat2_eeprom;
			}	

	sei();

	
		// end of loop--------------------
		// enable interrupt
		
			// loop for communication, and delay between reading temperatures
		for(j=0;j<interval;j++)
			{
			modbusGet();
			_delay_ms(10);
			}	//for(j
		
	PORTB ^= (1 << PB5); // set toggle
	}	// while
		//main
}


/// INTERRUPT ///

ISR(TIMER0_OVF_vect) { //this ISR is called 9765.625 times per second, CPU-20MHz
	modbusTickTimer();
}
 
