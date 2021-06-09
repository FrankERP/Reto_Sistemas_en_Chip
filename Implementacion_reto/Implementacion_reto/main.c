/*
 * Implementacion2.c
 *
 * Created: 07/06/2021 02:41:21 a. m.
 * Author : BitByBit
 */ 
#define F_CPU   8000000UL

#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>

#define DOUBLE_SPEED_MODE
#ifdef DOUBLE_SPEED_MODE
#define BAUD_PRESCALE (int)round(((((double)F_CPU / ((double)BAUDRATE * 8.0))) - 1.0)) //Definimos el prescaler del baud rate
#else
#define BAUD_PRESCALE (int)round(((((double)F_CPU / ((double)BAUDRATE * 16.0))) - 1.0))
#endif

////////////////////////////////////////////////////////////////////////////////////
#define NEXT_SONG 0X01
#define PREV_SONG 0X02
#define CMD_PLAY_W_INDEX 0X03 //Requiere número de canciòn
#define VOLUME_UP_ONE 0X04
#define VOLUME_DOWN_ONE 0X05
#define CMD_SET_VOLUME 0X06 //Requiere nivel de volumen entre 0 a 30
#define SET_DAC 0X17
#define CMD_PLAY_WITHVOLUME 0X22 //Requiere dos parámetros, volumen y  número de canción
#define CMD_SEL_DEV 0X09 //SELECT STORAGE DEVICE, DATA IS REQUIRED
#define DEV_TF 0X02 //HELLO,IM THE DATA REQUIRED
#define SLEEP_MODE_START 0X0A
#define SLEEP_MODE_WAKEUP 0X0B
#define CMD_RESET 0X0C
#define CMD_PLAY 0X0D
#define CMD_PAUSE 0X0E
#define CMD_PLAY_WITHFOLDER 0X0F //Requiere folder y número de canción
#define STOP_PLAY 0X16
#define PLAY_FOLDER 0X17 // Requiere folder 0x7E 06 17 00 01 XX EF
#define SET_CYCLEPLAY 0X19 //data is needed 00 start; 01 close
#define SET_DAC 0X17//data is needed 00 start DAC OUTPUT;01 DAC no output
////////////////////////////////////////////////////////////////////////////////////

#define CANCION_MAX 10

//Ambiente MP3
#define UNMUTE 0x40
#define MUTE 0x30
#define VOLUMEN_SUBIR 0x20
#define VOLUMEN_BAJAR 0x10
#define PLAY_NUM_CANCION 5
#define ANTERIOR 4
#define PLAY 3
#define PAUSE 2
#define SIGUIENTE 1
unsigned short int ctrlMP3Flag = 0; //Nibble más significativo = joystick, Nibble menos significativo = Matricial
static int8_t Send_buf[8] = {0} ;// Buffer para mandar comandos de 8 bits al mp3 (0x7E 06 CMD 00 xx yy EF)
unsigned short int num_song_cmd = 0;
void USART_Init(unsigned long BAUDRATE);
void USART_TxChar(char data);
void sendCommand(uint8_t command, uint8_t dat1, uint8_t dat2);
void controlMP3();
	
//Ambiente Teclado
#define BUZZER 5
unsigned short int song_buffer[2] = {0,2};   //------------------------------------------------------------------------------poner a 0
unsigned short int play_pause = 0;			// Saber si estamos en play o pausa, 1 = play, 0 = pausa
unsigned short int segundos_menu = 0;
unsigned short int valor_matricial;
void leerTeclado();
void procesarMatricial();

//Ambiente Joystick
unsigned short int adcBuffer = 0;
unsigned short int muteFlag = 0; // 0 = unmute, 1 = mute
unsigned short int adcFlag = 0;
unsigned short int mp3Cmd = 0;
	//Interrupción ADC
	//Interrupción externa 0, revisar

//Ambiente LCD
#define CHAR_MAX_LCD 16
#define ERROR 3
#define ESCOGER 2
#define NAVEGAR 1
#define REPRODUCIR 0
unsigned short int ctrlMenuFlag = 0;
unsigned short int buffer_estado = 0;
unsigned short int counter_b = 0;
unsigned short int numCancionActual = 0;
unsigned short int numCancionDisplay = 0;
unsigned short int contCorrimietos = 0;
unsigned short int nombreCompletado = 0;
unsigned short int renglon = 0x00; //En qué renglon ecribimos, 0x00 = Arriba, 0x40 = Abajo
void comandoLCD(char k);
void mostrarStringLCD(char a[]);
void mostrarCancion();
void mostrarDuracion();
void mandarChar();
unsigned short int binToBCD(unsigned short int bin);

//Ambiente LEDs
#define DELAY_LEDS_VOLUMEN 1
#define VOLUMEN_MAX 30
unsigned short int volumen = 10;
unsigned short int numLEDs_on = 0; //VOLUMEN MAX = 30, NUM LEDS = 10
void actualizarLEDsVolumen();

//TimerParaReproduccion
unsigned short int segundos = 0;
unsigned short int minutos = 0;
int contTimer = 1; // Tomams en cuenta el primer ciclo que se genera.
	//Interrupción de timer

//Tablas de canciones
char* canciones[] = {"19th Floor",
	"Black Catcher",
	"Edge of Eternity",
	"Find your way",
	"She no Dull",
	"Dream to live",
	"Fresh",
	"Roads",
	"Road Trip",
	"Worth a Try"};

char* duraciones[] = {"00:20",
	"00:25",
	"00:10",
	"00:15",
	"00:09",
	"00:11",
	"00:08",
	"00:14",
	"00:12",
	"00:20"};
	
uint8_t duracionesInt [10][2] = {{2,0},
	{3, 42},
	{3, 25},
	{2, 51},
	{3, 9},
	{3, 1},
	{3, 8},
	{2, 34},
	{3, 32},
	{3, 30}};

//MP3 
void USART_Init(unsigned long BAUDRATE){
	#ifdef DOUBLE_SPEED_MODE
		UCSRA |=(1 << U2X);
	#endif
	UCSRB |= (1 << RXEN) | (1 << TXEN); // Habilitar Rx y TX con interrupción de Rx
	UCSRC |= (1 << URSEL)| (1 << UCSZ0) | (1 << UCSZ1); // Comunicación de 8 bits con 1 bit de stop
	UBRRL = BAUD_PRESCALE;
	UBRRH = (BAUD_PRESCALE >> 8);
	_delay_ms(20);
}

void USART_TxChar(char data){
	while (!(UCSRA & (1<<UDRE)));
	UDR = data;
}

void sendCommand(uint8_t command, uint8_t dat1, uint8_t dat2){
	Send_buf[0] = 0x7e; //Byte de inicio
	Send_buf[1] = 0xff; //versión
	Send_buf[2] = 0x06; //Número de bytes del comando sin inicio ni final
	Send_buf[3] = command; //Comando
	Send_buf[4] = 0x00;//0x00 = no feedback, 0x01 = feedback
	Send_buf[5] = dat1;//data High
	Send_buf[6] = dat2; //data Low
	Send_buf[7] = 0xef; //Byte de fin
	cli(); // Apagamos interrupcines para no interrumpir el envio del comando
	for(uint8_t i=0; i<8; i++){
		USART_TxChar(Send_buf[i]) ;
		_delay_us(500);
	}
	sei();
}

void controlMP3(){
	//_delay_ms(50); //Se ejecuta cada 50 ms posiblemente 20 ms
	if((ctrlMP3Flag & 0xF0) == UNMUTE){
		sendCommand(CMD_SET_VOLUME, 0, volumen);
		}else if((ctrlMP3Flag & 0xF0) == MUTE){
		sendCommand(CMD_SET_VOLUME, 0, 0);
		}else if((ctrlMP3Flag & 0xF0) == VOLUMEN_SUBIR){
		sendCommand(VOLUME_UP_ONE, 0, 0);
		}else if((ctrlMP3Flag & 0xF0) == VOLUMEN_BAJAR){
		sendCommand(VOLUME_DOWN_ONE, 0, 0);
	}

	if((ctrlMP3Flag & 0x0F) == PLAY_NUM_CANCION){
		sendCommand(CMD_PLAY_W_INDEX, 0, (numCancionDisplay << 1)+1);
		}else if((ctrlMP3Flag & 0x0F) == PLAY){
		sendCommand(CMD_PLAY, 0, 0);
		}else if((ctrlMP3Flag & 0x0F) == PAUSE){
		sendCommand(CMD_PAUSE, 0, 0);
	}
	ctrlMP3Flag = 0;
}

//Teclado;
void leerTeclado(){
	valor_matricial = 3;
	unsigned short int lector;
	
	for(unsigned short int cont_filas = 0; cont_filas < 4 ; cont_filas++){
		PORTC = ~(1<<cont_filas);
		_delay_us(1);
		lector = PINC & 0xF0;
		if(lector==0xF0){
			valor_matricial += 4;
			}else{
			//    _delay_ms(1);
			//    lector = PINC & 0xF0;
			//    if(lector==0xF0)
			//     continue;
			if(lector>=0xE0){
				valor_matricial -= 3;
				}else if (lector >= 0xD0){
				valor_matricial -= 2;
				}else if(lector >= 0xB0){
				valor_matricial -= 1;
			}
			return;
		}
	}
	valor_matricial = 0xFF;
}

void procesarMatricial(){
	static unsigned short int displayMatriz[] = {1, 2, 3, 0xFF, 4, 5, 6, 0xFF, 7, 8 , 9, 10, 11, 0, 12, 13};
	//10 = Enter, 11 = play-pause, 12 = Anterior, 13 = Siguiente
	leerTeclado();
	if (valor_matricial < 16){
		valor_matricial = displayMatriz[valor_matricial];
		PORTA |= (1 << BUZZER);
		segundos_menu = 0;
		}else{
		return;
	}
	
	if(valor_matricial < 10){
		song_buffer[1] = song_buffer[0]; //Pasamos las unidades a las decenas
		song_buffer[0] = valor_matricial;
		ctrlMenuFlag = (1 << ESCOGER);
		}else if (valor_matricial == 10){ //Enter
		if (ctrlMenuFlag & (1 << ESCOGER)){
			numCancionDisplay = song_buffer[0] + 10*song_buffer[1];
			numCancionDisplay -= 1;
		}
		if ((ctrlMenuFlag & (1 << ESCOGER)) || (ctrlMenuFlag & (1 << NAVEGAR))){
			song_buffer[0] = 0;
			song_buffer[1] = 0;
			if (numCancionDisplay <= CANCION_MAX){
				ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | PLAY_NUM_CANCION;
				ctrlMenuFlag = (1 << REPRODUCIR);
				numCancionActual = numCancionDisplay;
				minutos = 0;
				segundos = 0;
				play_pause = 1;
				}else{
				ctrlMenuFlag = (1 << ERROR);
			}
		}
	}
	else if(valor_matricial == 11){ //Play-pause
		if (play_pause){
			ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | PAUSE;
			play_pause = 0;
			}else{
			ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | PLAY;
			play_pause = 1;
		}
		ctrlMenuFlag = (1 << REPRODUCIR);
		}else if(valor_matricial == 12){ //Anterior
		if (numCancionActual > 0){
			numCancionActual -= 1;
			}else{
			numCancionActual = CANCION_MAX-1;
			
		}
		ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | PLAY_NUM_CANCION;
		ctrlMenuFlag = (1 << REPRODUCIR);
		numCancionDisplay = numCancionActual;
		play_pause = 1;
		minutos = 0;
		segundos = 0;
		}else if(valor_matricial == 13){//Siguente
		if (numCancionActual < CANCION_MAX -1 ){
			numCancionActual += 1;
			
			}else{
			numCancionActual = 0;
		}
		ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | PLAY_NUM_CANCION;
		numCancionDisplay = numCancionActual;
		ctrlMenuFlag = (1 << REPRODUCIR);
		play_pause = 1;
		minutos = 0;
		segundos = 0;
	}
	if (PORTA & (1 << BUZZER)){
		_delay_ms(200);
		PORTA &= ~(1 << BUZZER);
	}
	
}
//ISR(ADC_vect){
void leerJoystick(){
	if (!(ADCSRA & (1 << ADIF)))
	return;
	adcBuffer = ADCL;
	adcBuffer = ADCH;
	if(!(ADMUX & 1)){ //REVISAMOS SI  ADMUX ES PAR PARA SABER QUÉ ADC TOCA REVISAR
		if(adcBuffer < 10){
			if((adcFlag & 3) != 2){ //Estado actual != Estado anterior 
				adcFlag = (adcFlag & 0XFE) | 2; //Estado = 10 Arriba
				ctrlMenuFlag = (1 << NAVEGAR);
				segundos_menu = 0;
				
				if (numCancionDisplay > 0){ //****************************
					numCancionDisplay -= 1;
				}
				//numCancionNav = (numCancionDisplay << 1)+1;
			}
		}else if(adcBuffer > 245){
			if((adcFlag & 3) != 0){ //Estado actual != Estado anterior 
				adcFlag = adcFlag & 0XFC;//Estado = 00 Abajo
				ctrlMenuFlag = (1 << NAVEGAR);
				segundos_menu = 0;
				
				if (numCancionDisplay < CANCION_MAX -1 ){ // Si no estamos en la penúltima canción (porque se ven dos canciones)
					numCancionDisplay += 1;
				}
				//numCancionNav = (numCancionDisplay << 1)+1;
			}
		}else{
			adcFlag = ((adcFlag & 0XFD) | 1);//Estado = 01 Medio
		}
		}else{
		if(adcBuffer < 10){
			if((adcFlag & 0XC) != 8){ //Estado actual != Estado anterior 
				adcFlag = (adcFlag & 0xFB) | 0X08; //Estado = 10 Izquierda
				muteFlag = 0;
				if (volumen){ //Si el volumen no es cero. Sin signo
					volumen -= 1;
					
					numLEDs_on = volumen/3;
                    ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | VOLUMEN_BAJAR;
				}
			}
		}else if(adcBuffer > 245){
			if((adcFlag & 0XC) != 0){ //Estado actual != Estado anterior 
				adcFlag = adcFlag & 0XF3; // Estado 00 = Derecha
				muteFlag = 0;
				if (volumen < VOLUMEN_MAX){
					volumen += 1;
					numLEDs_on = volumen/3;
					ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | VOLUMEN_SUBIR;
				}
			}
		}else{
			adcFlag = ((adcFlag & 0XF7) | 4); //Estado = 01 - Centro
		}
	}
	PORTD = 0xFF;
	ADMUX ^= 1; //CAMBIAMOS DE EJE PARA LA SIGUIENTE REVISIÓN
	ADCSRA |= (1 << ADSC);
}

ISR(INT0_vect){ //Revisar
	GICR = 0;//APAGAMOS LA INTERRUPCIÓN PARA QUE NO GUARDE LAS INTERRUPCIONES DE LOS REBOTES. Se prende de nuevo en el timer
	ADCSRA &= ~(1 << ADIE) & ~(1 << ADSC); // Apagams la interrupción de ADC para que no interfiera con el mute
	if(muteFlag){
		muteFlag = 0;
		ctrlMP3Flag = (ctrlMP3Flag & 0x0F) |  UNMUTE;
		}else{
		muteFlag = 1;
		ctrlMP3Flag = (ctrlMP3Flag & 0x0F) |  MUTE;
	}
}

//LCD
void comandoLCD(char k){
	PORTB = k;
	PORTA = (PORTA | 0x01) & 0xF9;//RS = 0, RW = 0, EN = 1
	_delay_ms(2);
	PORTA = PORTA & 0xF8;// EN = 0
}

void mostrarStringLCD(char a[]){ //Strings > 27 chars -> tiempo > 1ms
	comandoLCD(0x80 | renglon); // Mueve el cursor a la línea de "renglon"
	for(int i = 0; i < strlen(a); i++ ){
		PORTB = a[i];
		PORTA = 0x05; //RS = 1, RW = 0, EN = 1;
		_delay_us(36);
		PORTA = 0x04; //RS = 1, RW = 0, EN =0;
		if(i == 15)
		comandoLCD(0xC0);//Mueve el cursor a la segunda línea
	}
}

void mostrarCancion(){
	if (numCancionDisplay < 0 || numCancionDisplay >= CANCION_MAX)
		return;
	unsigned short int corrimietos = 0;
	if (strlen(canciones[numCancionDisplay]) >= CHAR_MAX_LCD)
		corrimietos = contCorrimietos;
	comandoLCD(0x80 | renglon);
	unsigned short int decena = 0;
	unsigned short int numCharLCD = 0;
	nombreCompletado = 1;
	while (numCharLCD + corrimietos < strlen(canciones[numCancionDisplay]) + 3 + decena && numCharLCD <= CHAR_MAX_LCD){
		if(numCharLCD == 0){
			if (numCancionDisplay >= 9){
				unsigned short int temp = binToBCD(numCancionDisplay + 1); // + 1 PORQUE EMPIEZA EN 0
				PORTB = (temp >> 4) + 48; // + 48 PARA PASAR A SU VALOR ASCII
				mandarChar();
				numCharLCD += 1;
				decena += 1;
				PORTB = (temp & 0x0F) + 48;
				
				}else{
				PORTB = numCancionDisplay + 49; // PASAMOS  numCancionActual A SU VALOR ASCII + 1, PORQUE EMPIEZA EN 0
			}
		}else if(numCharLCD == 1 + decena){ //numCancionActual EMPIEZA EN 0
			PORTB = '.';
		}else if(numCharLCD == 2 + decena){
			PORTB = ' ';
		}else if(numCharLCD < CHAR_MAX_LCD){
			PORTB = canciones[numCancionDisplay][numCharLCD + corrimietos - 3 - decena];
		}else{
			contCorrimietos += 1;
			PORTB = canciones[numCancionDisplay][numCharLCD + (++corrimietos) - 3 - decena];
			nombreCompletado = 0;
		}
		if (nombreCompletado)
			contCorrimietos = 0;
		numCharLCD += 1;
		mandarChar();
	}
}

void mostrarDuracion(){
	comandoLCD(0xC0); //Pasamos a la segunda linea nos movemos a la columna necesaria según los corriemientos del nombre de la canción
	unsigned short int temp;
	temp = binToBCD(minutos);
	PORTB = (temp >> 4) + 48;
	mandarChar();
	PORTB = (temp & 0x0F) + 48;
	mandarChar();
	PORTB = ':';
	mandarChar();
	temp = binToBCD(segundos);
	PORTB = (temp >> 4) + 48;
	mandarChar();
	PORTB = (temp & 0x0F) + 48;
	mandarChar();
	PORTB = ' ';
	for (unsigned short int i = 0; i < 6; i++){
		mandarChar();
	}
	for( unsigned short int i = 0; i < 5; i++){
		PORTB = duraciones[numCancionDisplay][i];
		mandarChar();
	}
}

unsigned short int binToBCD(unsigned short int bin){ //bin en [0, 99] -> BCD en dos nibbles
	unsigned short int bcd = 0;
	bcd = bin / 10;
	bcd = (bcd << 4);
	bcd += bin % 10;
	return bcd;
}

void mandarChar(){
	PORTA = (PORTA | 0x05) & 0xFD;//RS  = 1, RW = 0, EN = 1
	_delay_us(40);
	PORTA = (PORTA | 0x04) & 0xFC;//RS = 1, RW = 0, EN =0
}

void mostrarMenu(){
	if(ctrlMenuFlag & (1 << ESCOGER)){ // Menú ESCOGER
		comandoLCD(0x01);
		renglon = 0x40;
		unsigned short int temp = song_buffer[0] + 10* song_buffer[1];
		if (temp <= CANCION_MAX && temp != 0){
			numCancionDisplay = temp - 1;
			mostrarCancion();
		}
		renglon = 0x00;
		mostrarStringLCD("Cancion: ");
		PORTB = song_buffer[1] + 48;
		mandarChar();
		PORTB = song_buffer[0] + 48;
		mandarChar();
	}else if(ctrlMenuFlag & (1 << NAVEGAR)){ // Menú NAVEGAR
		comandoLCD(0x01);
		renglon = 0x00;
		mostrarCancion();
		renglon = 0x40;
		numCancionDisplay ++;
		mostrarCancion();
		numCancionDisplay --;
		
	}else if(ctrlMenuFlag & (1 << REPRODUCIR)){ // Menú REPRODUCIR
		comandoLCD(0x01);
		renglon = 0x00;
		numCancionDisplay = numCancionActual;
		mostrarCancion();
		mostrarDuracion();
	}else{ // Menú ERROR
		comandoLCD(1);
		renglon = 0x00;
		mostrarStringLCD("Error");
	}
	
	if((buffer_estado == ctrlMenuFlag) && (segundos_menu == 3)){
		comandoLCD(0x01);
		ctrlMenuFlag = (1 << REPRODUCIR);
		numCancionDisplay = numCancionActual;
		segundos_menu = 0;
	}
			
	buffer_estado = ctrlMenuFlag;
}

//LEDsVolumen
unsigned short int posLED = 0;
void actualizarLEDsVolumen(){
	if (muteFlag)
	return;
	unsigned short int numLEDs_on = volumen/3; //VOLUMEN MAX = 30, NUM LEDS = 10
	if (posLED < numLEDs_on){
		PORTD |= 0xF8;
		if (posLED < 5){
			PORTA = (PORTA | 8) & 0xEF;
			}else{
			PORTA = (PORTA | 0x10) & 0xF7;
		}
		PORTD = (PORTD | (~(1 << ((posLED%5)+3)) & 0xF8)) & (~(1 << ((posLED%5)+3))| 0x07);
		posLED++;
		}else{
		posLED = 0;
	}
}

//Timer 1ms
ISR(TIMER0_COMP_vect){
	if (!muteFlag)
		actualizarLEDsVolumen(); 
	if (contTimer > 1000){ // 1 s
		contTimer = 0;
		if (play_pause){
			segundos += 1;
			if(segundos == 60){
				minutos += 1;
				segundos = 0;
			}
			if((((duraciones[numCancionActual][0]-48)*10) + (duraciones[numCancionActual][1]-48) == minutos) && (((duraciones[numCancionActual][3]-48)*10) + (duraciones[numCancionActual][4]-48)) +1 == segundos){
				ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | SIGUIENTE;
				numCancionActual += 1;
				numCancionDisplay +=1;
				minutos = 0;
				segundos = 0;
				if(numCancionActual == CANCION_MAX){
					ctrlMP3Flag = (ctrlMP3Flag & 0xF0) | PLAY_NUM_CANCION;
					numCancionActual = 0;
					numCancionDisplay = 0;
				}
			}
		}
		if(ctrlMenuFlag != (1 << REPRODUCIR)){
			segundos_menu ++;
			}else{
			segundos_menu = 0;
		}
		if (GICR == 0){
			GICR = (1 << INT0);
			ADCSRA |= (1 << ADIE);
		}
		if (contCorrimietos != 0){
			contCorrimietos++;
		}
	}else{
		contTimer++;
	}
}

int main(void){
	//Puertos
	DDRA = 0x3F;
	DDRB = 0xFF;
	DDRC = 0x0F;
	PORTC = 0xF0; //Inputs pull-up para teclado matricial
	DDRD = 0xFE;
	PORTD = 0x04; //Input pull-up para SW de Joystick
	
	//Palabras de control
	//Timer 500 us para el refrescamiento de los leds
	TIMSK = (1 << OCIE0);
	OCR0 = 125;
	TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
	
	//Interrupcion Externa
	GICR = (1 << INT0); 
	MCUCR = (1 << ISC01); //Flanco de bajada
	
	//ADC
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX2) | (1 << MUX1);
	ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	//USART
	USART_Init(9600);
	
	//LCD
	comandoLCD(0x01); //Limpia la pantalla
	comandoLCD(0x06); //Configura la dirección del cursor
	comandoLCD(0x0C); //Habilita la pantalla y deshabilita el cursor
	comandoLCD(0x38); // Configura función de 8 bits para dos lineas, 5x7 puntos
	ctrlMenuFlag = (1 <<REPRODUCIR);
	
	//LEDs
	
	volumen = 10;
	numLEDs_on = volumen/3;
	PORTA = (PORTA | 8) & 0xEF;
	sendCommand(CMD_PLAY_WITHVOLUME, 10, 0x01);
	_delay_ms(50);
	sendCommand(PAUSE, 0,0);
	
	//Inicio
 	mostrarStringLCD("    BitByBit");
	renglon = 0x40;
	mostrarStringLCD("      MP3");
	renglon = 0;
	_delay_ms(1000);
	comandoLCD(1);
	mostrarStringLCD("      Raul");
	renglon = 0x40;
	mostrarStringLCD("     Navarro");
	renglon = 0;
	_delay_ms(1000);
	comandoLCD(1);
	mostrarStringLCD("   Francisco");
	renglon = 0x40;
	mostrarStringLCD("     Rocha");
	renglon = 0;
	_delay_ms(1000);
	comandoLCD(1);
	mostrarStringLCD("      Alex");
	renglon = 0x40;
	mostrarStringLCD("     Flores");
	renglon = 0;
	_delay_ms(1000);
	comandoLCD(1);
	
	ADCSRA |= (1 << ADSC);
	sei();

   while (1) {
	   
	   //controlMP3(); // duración_MAX > 62 ms
	   
	   if (contTimer % 200 < 10)
	   mostrarMenu();
	   
	   if (contTimer % 300 < 10)
	   controlMP3();
	   
	   if (contTimer % 100 < 10)
	   leerJoystick();
	   
	   if (contTimer % 100 < 10)
	   procesarMatricial();
	   
   }
}

