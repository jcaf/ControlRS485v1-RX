/*
 * main.c
 *
 *  Created on: Feb 8, 2022
 *      Author: jcaf
 *
 *      1) avrdude -c usbasp -B5 -p m328 -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xfd:m
 *      2) avrdude -c usbasp -B5 -p m328 -U flash:w:ControlRS485v1-RX.hex
 *
 */

#include "main.h"
#include "serial/serial.h"
#include "usart/usart.h"
#include "adc/adc.h"
#include "system.h"
#include "types.h"

volatile struct _isr_flag
{
	unsigned sysTickMs :1;
	unsigned __a :7;
} isr_flag = { 0 };
struct _mainflag mainflag;

int8_t stateButtonA;
int8_t stateButtonB;
int8_t stateButtonC;
int8_t stateButtonD;
uint16_t rv_light;
uint16_t rv_motor;


#define PWM_TOP 255
#define PWM_BOTTOM 0

#define ADC_TOP 1023
#define ADC_BOTTOM 0

/* ***************************** */
//ideal
//#define MOTOR_VDC_MAX 12.0f
//#define MOTOR_VDC_MIN 5.0f

//LD298D tine sus caidas de tension internas
#define MOTOR_VDC_MAX 10.51f	//@DC=255
//#define MOTOR_VDC_MIN (MOTOR_VDC_MAX-7.0f) //5.0f
#define MOTOR_VDC_MIN (0.0f) //5.0f


#define MOTOR_DUTY_MAX PWM_TOP
#define MOTOR_DUTY_MIN ((MOTOR_VDC_MIN*PWM_TOP)/MOTOR_VDC_MAX)

/* calculate the slope m*/
#define MOTOR_FX_M ( (MOTOR_DUTY_MAX - MOTOR_DUTY_MIN)/ (ADC_TOP-ADC_BOTTOM))
/* calculate the B*/
#define MOTOR_FX_B MOTOR_DUTY_MIN

/* ***************************** */
#define LIGHT_VDC_MAX 24.0f
#define LIGHT_VDC_MIN 12.0f

#define LIGHT_DUTY_MAX PWM_TOP
#define LIGHT_DUTY_MIN ((LIGHT_VDC_MIN*PWM_TOP)/LIGHT_VDC_MAX)

/* calculate the slope m*/
#define LIGHT_FX_M ( (LIGHT_DUTY_MAX - LIGHT_DUTY_MIN)/ (ADC_TOP-ADC_BOTTOM))
/* calculate the B*/
#define LIGHT_FX_B LIGHT_DUTY_MIN

/* ***************************** */
#define OUTX_OUTY_DELAY_TRANSITION_MS 4// 10ms * 4 = 40 ms
////////////////////////////////////////////////////////////////////


uint8_t checksum(char *str, uint8_t length);
int8_t str_trimlr(char *str_in, char *str_out, char l, char r);
void rx_trama(void);

char bin_to_asciihex(char c)//nibbleBin_to_asciiHex
{
    if (c < 10)
        return c+'0';
    else
        return (c-10) + 'A';
}

/* Button assert in Low
 * */
int8_t last_stateButtonA;
int8_t last_stateButtonB;
int8_t last_stateButtonC;
int8_t last_stateButtonD;

void parseButtonA(void)
{
	static int8_t toggle;
	//static int8_t sm0;

	if (last_stateButtonA != stateButtonA)
	{
		last_stateButtonA = stateButtonA;

		if (stateButtonA == 0)
		{
			toggle = !toggle;
			if (toggle == 1)
			{
				PinTo1(PORTWxRELAY_TK_12V, PINxRELAY_TK_12V);
			}
			else
			{
				PinTo0(PORTWxRELAY_TK_12V, PINxRELAY_TK_12V);
			}
		}
	}
}


/*
 * despues de haber presionado B, la salida actual pasa a 0, y despues de 40ms la otra salida se activa.
 * outX inicia a activo.
 * */

void parseButtonB(void)
{
	static int8_t sm0;
	static int8_t toggle;
	static uint8_t counter;

	if (sm0 == 0)
	{
		if (last_stateButtonB != stateButtonB)
		{
			last_stateButtonB = stateButtonB;

			if (stateButtonB == 0)
			{
				toggle = !toggle;

				if (toggle == 1)
				{
					PinTo0(PORTWxOUT_X, PINxOUT_X);
				}
				else
				{
					PinTo0(PORTWxOUT_Y, PINxOUT_Y);
				}
				sm0++;
			}
		}
	}
	else
	{
		if (mainflag.sysTickMs)//unidades de  10 ms ingresa -> 10ms * 4 = 40 ms
		{
			if (++counter >= OUTX_OUTY_DELAY_TRANSITION_MS)
			{
				counter  = 0 ;

				if (toggle == 1)
				{
					PinTo1(PORTWxOUT_Y, PINxOUT_Y);
				}
				else
				{
					PinTo1(PORTWxOUT_X, PINxOUT_X);
				}
				sm0 = 0;
			}
		}
	}
}
void PWM_DC_OCR0B_ZERO(void)
{
	PinTo0(PORTWxMOTOR_PWM, PINxMOTOR_PWM);
	TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) |  (1 << WGM01) | (1 << WGM00);//OC0B disconnected
	PinTo0(PORTWxMOTOR_PWM, PINxMOTOR_PWM);
}
void parseButtonB_C(void)
{
	if ( (stateButtonC == 1) && (stateButtonD == 1) )
	{
		//Motor off, denergized
		//OCR0B = 0;//DC motor
		PinTo0(PORTWxMOTOR_DIR_A, PINxMOTOR_DIR_A);
		PinTo0(PORTWxMOTOR_DIR_B, PINxMOTOR_DIR_B);
		PWM_DC_OCR0B_ZERO();
	}
	else if ( (stateButtonC == 0) && (stateButtonD == 1) ) //turn right
	{
		rv_motor = ADC_read(ADC_CH_0);
		TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) |  (1 << WGM01) | (1 << WGM00);//OC0B disconnected
		OCR0B = (MOTOR_FX_M*rv_motor) + MOTOR_FX_B;
		//
		PinTo1(PORTWxMOTOR_DIR_B, PINxMOTOR_DIR_B);
		PinTo0(PORTWxMOTOR_DIR_A, PINxMOTOR_DIR_A);
	}
	else if ( (stateButtonC == 1) && (stateButtonD == 0) ) //turn right
	{
		rv_motor = ADC_read(ADC_CH_0);
		TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) |  (1 << WGM01) | (1 << WGM00);//OC0B disconnected
		OCR0B = (MOTOR_FX_M*rv_motor) + MOTOR_FX_B;

		PinTo0(PORTWxMOTOR_DIR_B, PINxMOTOR_DIR_B);
		PinTo1(PORTWxMOTOR_DIR_A, PINxMOTOR_DIR_A);
	}
}


int main(void)
{
	int8_t c = 0;

	PORTB=PORTC=PORTD = 0;

	ConfigOutputPin(CONFIGIOxMOTOR_DIR_B, PINxMOTOR_DIR_B);
	ConfigOutputPin(CONFIGIOxMOTOR_DIR_A, PINxMOTOR_DIR_A);
	ConfigOutputPin(CONFIGIOxRELAY_TK_12V, PINxRELAY_TK_12V);
	ConfigOutputPin(CONFIGIOxRS485_DIR, PINxRS485_DIR);//RX driver enable
	ConfigOutputPin(CONFIGIOxMOTOR_PWM, PINxMOTOR_PWM);
	ConfigOutputPin(CONFIGIOxLIGHT_PWM, PINxLIGHT_PWM);

	//
	last_stateButtonA = stateButtonA;
	last_stateButtonB = stateButtonB;

	/* OUTX inicia a positivo */
	PinTo1(PORTWxOUT_X, PINxOUT_X);
	ConfigOutputPin(CONFIGIOxOUT_X, PINxOUT_X);
	ConfigOutputPin(CONFIGIOxOUT_Y, PINxOUT_Y);

	USART_Init ( (uint16_t) MYUBRR );
	ADC_init(ADC_MODE_SINGLE_END);

	/* Config PWM on OC0A y OC0B*/
	TCNT0 = 0;
	OCR0A = 0;//DC light
	OCR0B = 0;//DC motor
	//TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) |  (1 << WGM01) | (1 << WGM00);//Fast PWM MODE
	PWM_DC_OCR0B_ZERO();

	TCCR0B = (0 << WGM02) | (0 << CS02) | (0 << CS01) | (1 << CS00);	//Preescaling = 1
	//TCCR0B = (0 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00);	//Preescaling = 8

	PinTo1(PORTWxMOTOR_DIR_B, PINxMOTOR_DIR_B);
	PinTo0(PORTWxMOTOR_DIR_A, PINxMOTOR_DIR_A);

//do
//{
//	rv_motor = ADC_read(ADC_CH_0);
//
//	//Set motor Duty Cycle
//	/* MOTOR DC Voltage in range of 5 - 12 */
//	OCR0B = (MOTOR_FX_M*rv_motor) + MOTOR_FX_B;
//
//	/* Light DC Voltage in range of 12 - 24 */
//	OCR0A = (LIGHT_FX_M*rv_light) + LIGHT_FX_B;
//
//}while(1);


	/*********************/
	//Config TCN1 to 10ms
	TCNT1 = 0x00;
	TCCR1A = 0;
	TCCR1B =  ( 1<< WGM12 ) | (1 << CS12) | (0 << CS11) | (1 << CS10); 	//CTC, PRES=1024
	OCR1A = CTC_SET_OCR_BYTIME(10e-3, 1024);			//10ms PRES 1024 -> OCRA0 = 107
	//--> SYSTICK = 10e-3
	TIMSK1 |= (1 << OCIE1A);
	sei();
	//
	//while (1);

/*
 *
	//Config to 1ms
	TCNT0 = 0x00;
	TCCR0A = (1 << WGM01);
	TCCR0B =  (1 << CS02) | (0 << CS01) | (1 << CS00); 	//CTC, PRES=1024
	OCR0A = CTC_SET_OCR_BYTIME(10e-3, 1024);			//10ms PRES 1024 -> OCRA0 = 107
	//--> SYSTICK = 10e-3
	TIMSK0 |= (1 << OCIE0A);
	sei();
	//
 */

	while (1)
	{
		if (isr_flag.sysTickMs)
		{
			isr_flag.sysTickMs = 0;
			mainflag.sysTickMs = 1;
		}
		//----------------------------------
		if (mainflag.sysTickMs)
		{
			if (++c >= (20/SYSTICK_MS) )
			{
				c = 0;
				//
				rx_trama();
				parseButtonA();
				//
//				rv_motor = ADC_read(ADC_CH_0);
//				/* MOTOR DC Voltage in range of 5 - 12 */
//				OCR0B = (MOTOR_FX_M*rv_motor) + MOTOR_FX_B;

				/* Light DC Voltage in range of 12 - 24 */
				OCR0A = (LIGHT_FX_M*rv_light) + LIGHT_FX_B;

				parseButtonB_C();
			}
		}

		parseButtonB();
		//----------------------------------
		mainflag.sysTickMs = 0;
	}
}

//--> SYSTICK = 10e-3
//ISR(TIMER0_COMPA_vect)
//{
//	isr_flag.sysTickMs = 1;
//}

ISR(TIMER1_COMPA_vect)
{
	isr_flag.sysTickMs = 1;
	//PinToggle(PORTWxMOTOR_PWM, PINxMOTOR_PWM);
}


/*
 * */
uint8_t checksum(char *str, uint8_t length)
{
    uint16_t acc = 0;
    for (int i=0; i< length; i++)
    {
        acc += str[i];
    }
    return (uint8_t)(acc);
}


/* recorre todo el array
 * 1: success
 * 0: fail
 */

int8_t str_trimlr(char *str_in, char *str_out, char l, char r)
{
	int8_t counter = 0;
	int8_t idx = 0;
	int8_t length = strlen(str_in);

	for (counter = 0; counter < length; counter ++)
	{
		if (str_in[counter] == l)
		{
			counter ++;//sale dejando apuntando al siguiente byte

			//@N512F
			//copy

			for (;counter < length; counter++)
			{

				if (str_in[counter] == r)
				{
					//ok, hasta aqui nomas, procede
					str_out[idx] = '\0';//fin de cadena trimmed
					return 1;
				}

				str_out[idx++] = str_in[counter];
			}
		}
	}

	return 0;
}

/*
 * la busqueda en el buffer circular es cada "x" ms
 * no puede ser directa porque perderia mucho tiempo hasta que se complete la trama completa
 *
 * octave:7>  dec2hex(sum(int8('@N512F1023R256')))
	ans = 321 -> el resultado esta en HEX, solo me quedo con el byte menor = 0x21
 *
 * 	@N512F1023R256C21
	@N512F1023R257C22
 */

/*
 * //Construir payload data + checksum
 @NxxxxFxxxxRxxxxCcc'\r\n'
 1234546789........64

 */

struct _rx
{
	int8_t sm0;
	//#define RX_BUFFER_MAXSIZE 32
	//char buffer[RX_BUFFER_MAXSIZE];
	char buffer[SCIRBUF_BUFF_SIZE+1];//+1 solo si se convertira en C-str
}rx;

void rx_trama(void)
{
	char *ptr;
	//
	uint8_t checks = 0;
	char *pb = rx.buffer;
	uint8_t counter = 0;
	char buff_temp[20];
	int8_t idxtokens = 0;
	int8_t idx_base = 0;
	uint8_t bytes_available;
	static int CstrIdx = 0;
	char c;

	#define TOKENS_NUMMAX 7

	const char tokens[TOKENS_NUMMAX] = {'@','R','A','B','C','D',0x0D};//Enter x proteus

	char buff_out[SCIRBUF_BUFF_SIZE];
	//
	#define CSTR_SIZEMAX SCIRBUF_BUFF_SIZE
	static char Cstr[CSTR_SIZEMAX];
	static int CstrCurrentAvailableSpace = CSTR_SIZEMAX;
	int nbytes2write=0;

	if (rx.sm0 == 0)
	{
		bytes_available = scirbuf_bytes_available();
		if (bytes_available > 0)
		{
			scirbuf_read_nbytes((uint8_t*)buff_out, bytes_available);

			if (CstrCurrentAvailableSpace == CSTR_SIZEMAX)
			{
				CstrIdx = 0x00;
			}
			int temp = CstrCurrentAvailableSpace - bytes_available;
			if (temp < 0)//NO HAY ESPACIO
			{
				nbytes2write = CstrCurrentAvailableSpace;
				CstrCurrentAvailableSpace = CSTR_SIZEMAX;//RESET AL MAX para la sgte pasada
			}
			else
			{
				nbytes2write = bytes_available;
				CstrCurrentAvailableSpace = temp;
			}
			for (int i=0; i<nbytes2write; i++)
			{
				Cstr[CstrIdx++] = buff_out[i];
				//if (++CstrIdx > MAX)
				//{CstrIdx	= 0;}
			}

			//Ahora analizo por toda la trama completa @NxxxxFxxxxRxxxxCcc'\r\n'
			idxtokens = 0;
			idx_base = 0;
			for (int i=0; i < CstrIdx; i++)
			{
				c = Cstr[i];
				if (  c == tokens[idxtokens] )
				{
					idxtokens++;
				}
				//
				if (idxtokens > 0) //osea se encontró al menos el primer token '@'
				{
					rx.buffer[idx_base++] = c;//empieza a guardar los datos
				}

				if (idxtokens >= TOKENS_NUMMAX)
				{
					//Todos los tokens fueron encontrados de principio a fin
					rx.buffer[idx_base] = '\0';//fin, convierte a Cstring

					//reinicializar Cstr
					CstrCurrentAvailableSpace = CSTR_SIZEMAX;//strcpy(Cstr, "");//reset Cstr;
					//
					rx.sm0++;

					break;
				}
			}
		}
	}
	if (rx.sm0 == 1)
	{

		rx.sm0 = 0x00;//si falla cualquiera de las condiciones subsiguientes, regresa al estado anterior

		//en este punto tengo la probabilidad de tener el buffer correctamente copiado desde el Buffer circular
		//Necesito tener si o si toda la trama desde @ hasta \r\n.. OK
		//@NxxxxFxxxxRxxxxCcc'\r\n'
		counter = 0x00;
		pb = &rx.buffer[0];

		while (*pb != 'X')//Si recorre todo array y no encuentra C, entonces resetear algoritmo de busqueda
		{
			pb++;
			counter++;
			//add
			if (counter >= CstrIdx)
			{
				return;
			}
		}
		checks = checksum(rx.buffer, counter);//checksum desde @....X, nada mas

		//PinTo1(PORTWxLED1, PINxLED1);
		//itoa(checks, str, 10);
		//sprintf(str,"%d",checks);

		//0x ASCII-HEX CODED
//		if (rx.buffer[counter+1] == str[0])
//		{
//			if (rx.buffer[counter+2] == str[1])
		if (rx.buffer[counter+1] == bin_to_asciihex(checks>>4))
		{
			if (rx.buffer[counter+2] == bin_to_asciihex(checks & 0x0F))
			{
				//ok, ambos checksum son iguales
				//-> se usa toda la data integra
				//Convertir de ASCII a integer, separar la data
				if (str_trimlr(rx.buffer, buff_temp, 'R', 'A' ))
				{

					//rv_light = atoi(buff_temp);
					rv_light = (uint16_t)strtol(buff_temp, &ptr, 10);

				}
				if (str_trimlr(rx.buffer, buff_temp, 'A', 'B' ))
				{
					//stateButtonA = atoi(buff_temp);
					stateButtonA = (uint8_t)strtol(buff_temp, &ptr, 10);

				}
				if (str_trimlr(rx.buffer, buff_temp, 'B', 'C' ))
				{
					//stateButtonB = atoi(buff_temp);
					stateButtonB = (uint8_t)strtol(buff_temp, &ptr, 10);
				}
				if (str_trimlr(rx.buffer, buff_temp, 'C', 'D' ))
				{
					//stateButtonC = atoi(buff_temp);
					stateButtonC = (uint8_t)strtol(buff_temp, &ptr, 10);
				}
				if (str_trimlr(rx.buffer, buff_temp, 'D', 'X' ))
				{
					//stateButtonD = atoi(buff_temp);
					stateButtonD = (uint8_t)strtol(buff_temp, &ptr, 10);
				}


				rx.sm0 = 0x00;

			}
		}
	}
}
