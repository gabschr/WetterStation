//#include <avr/io.h>
//#include <avr/interrupt.h>

//Maximalwerte
#define MAXZAEHL 65000

// GLOBALE VARIABLEN
double wetterSensor[3]; //Sensor fuer Temp und Feuchtigkeit (0. Wert: Zeitstempel (ab Start vom Arduino in us), 1.Wert: Feucht, 2.Wert: Temp
unsigned long abrufIntervallSekunden = 3;
unsigned long letzteMesszeitpunkt = 0;

uint8_t timer0_over = 0; // Timer0 Zeitueberschreitung
volatile bool timer1_over = 0; // Timer1 Zeitueberschreitung
bool tasterVorher = 0;
volatile uint8_t timer2_over = 0; //Timer2 Zeitueberschreitung
volatile uint8_t portInterruptPD3 = 0;

//byte portWert = 0;	//Wert des Portes

// PORTDEKLARATIONEN
#define ledrtPD	PB5			//PIN fuer rote LED
#define ledgePD	PB4			//PIN fuer geldbe 
#define taster  PB0			//PIN fuer Taster zum Umschalten der Sensoren (PIN8)
#define feuchtLuftPD  PD3	//PIN fuer den Feuchtigkeits- und Temp- Sensor (PIN3)
#define dht22Sensor   PD4	//PIN fuer DHT22-Sensor (Feuchtigkeit+Temp)

uint32_t t = 0;				//Zeiterfassunug

// Fehlerauswertung
int16_t fehler = 0;

// ######################## INTERRUPT SERVICE ROUTINEN ###############################
// Routine zum Abfragen der Taster
ISR(PCINT0_vect) {
	byte portWert = 0;
	uint8_t aktuellerTasterWert;

	portWert = PINB;	//welcher Port wurde angesprochen? (8 BIT- pro Taster ein Bit)
	// wurde Taster gedrueckt?
	aktuellerTasterWert = (portWert & (1 << taster) >> taster);
	if (aktuellerTasterWert == 1) {	//wurde taster gedrueckt?
		// entprellen
		if (tasterVorher == 0) {	// Taster wurde zum ersten Mal gedrueckt
			tasterVorher = 1;
			timer0_over = 0;
			return;
		}
		else {
			if (timer0_over <= 157) {	//1 Zaehldurchlauf ca. 128 Mikrosekunden
				return;
			}
			tasterVorher = 0;
		}
		PORTB ^= (1 << ledgePD);	//LED ge toggelnd
		Serial.println("TASTER gedrueckt");
		TCCR0B = 0;
	}
}

// Routine zum Abfragen der Sensoren
ISR(PCINT2_vec) { //Sensoren
	byte portWert = 0;
	byte statusFeuchtLuftPD = 0;	//Status des Sensors abfragen
	uint8_t sensorBitWert;  //Variable zum Festhalten des aktuellen Bits am Sensor

	portWert = PIND;	//welcher Port wurde angesprochen? (8 BIT- pro Taster ein Bit)
	statusFeuchtLuftPD = (portWert & (1 << feuchtLuftPD) >> feuchtLuftPD);	//PORT vom Sensor ermitteln

	Serial.println("Interrupt Sensor");

	if (statusFeuchtLuftPD != 0) {
		if (portInterruptPD3 == 0) {
			portInterruptPD3 = 1;
			return;
		}
		else {
			portInterruptPD3 = 0;
			return;
		}
	}

}

ISR(INT1_vect)
{
	portInterruptPD3++;
	PORTB ^= (1 << ledgePD);
	//Serial.println(portInterruptPD3);
}

// Timer0 (Taster)
ISR(TIMER0_OVF_vect__vector_16) {
	timer0_over++;
}

// Timer1 (langsames Blinken)
ISR(TIMER1_COMPA_vect) {
	PORTB ^= (1 << ledrtPD);  // LED toggelnd
	timer1_over++;
	OCR1A -= 1000;
	OCR1A = OCR1A % 65535;
}

// Timer2 (Taster)
ISR(TIMER2_COMPA_vect) {
	timer2_over++;
	//Serial.print("TIMER2: ");
	//Serial.println(timer2_over);
}


// ################ SETUP-ROUTINE ###########################
void setup() {
	Serial.begin(9600);
	Serial.println("Hallo");

	// Deklarieren der I-O-Ports und Setzen der Pegel
	DDRB |= (1 << ledrtPD);     // als Ausgang
	PORTB |= (1 << ledrtPD);	//HIGH setzen
	DDRB |= (1 << ledgePD);		// als Ausgang
	PORTB |= (1 << ledgePD);	//HIGH setzen
	DDRB &= ~(1 << taster); // als Eingang setzen - LOW-aktiv 
	PORTB |= (1 << taster); // Pull-Up-Widerstand am Arduino setzen
	DDRD |= (1 << feuchtLuftPD);  // als Ausgang setzen
	DDRD |= (1 << dht22Sensor); // als Ausgang setzen

	// ############## INTERRUPTS ######################
	cli();	//deaktivieren von Interrupts

	// ############### TIMER INTERRUPTS ####################
	//TIMER0 fuer Entprellen der Taster
	//TCCR0A = 0; // set TCCR1A register to 0
	//TCCR0B = 0; // set TCCR1B register to 0
	//TCNT0 = 6; // reset counter value
	//TCCR0B |= (1 << CS01);
	//TIMSK0 |= (1 << TOIE0); // Ueberlauf Modus

	// TIMER1 fuer langsames Blinken der LED
	TCCR1A = 0; // set TCCR1A register to 0
	TCCR1B = 0; // set TCCR1B register to 0
	TCNT1 = 0; // reset counter value
	OCR1A = 65535;	//MAX bei 16-BIT-TIMER: 65535, bei 8-Bit 2 hoch 8 - 1 = 255
	TCCR1B |= (1 << CS12) | (1 << CS10);
	TCCR1B |= (1 << WGM12); // turn on CTC mode
	TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
	
	////TIMER2 fuer Sensorabfrage
	TCCR2A = 0; // set TCCR1A register to 0
	TCCR2B = 0; // set TCCR1B register to 0
	TCNT2 = 0; // reset counter value
	OCR2A = 124;
	TCCR2B |= (1 << CS21) | (1 << CS20);
	TCCR2B |= (1 << WGM22); // CTC MODUS einstellen
	TIMSK2 |= (1 << OCIE2A); // timer compare Modus

	// ############## EXTERNAL INTERRUPTS ######################
	// Taster auf PORTB
	PCICR |= (1 << PCIE0);		// PIN CHANGE INTERRUPT FUER PB-Gruppe (TASTER)
	PCMSK0 |= (1 << PCINT0);	//Externer Interruppt fuer Port PD2

	// Sensoren auf PORTD
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT2);

	//PORT PD3
	EICRA |= (1 << ISC10); //fallende und steigende Flanke erzeugt einen Interrupt
	EIMSK |= (1 << INT1);
	
	sei();	//aktivieren von Interrupts
}

void loop(){
	millis();
	sensorFeuchtTempAbfrage(feuchtLuftPD);
	if (timer1_over ==1) {
		Serial.println(timer1_over);
		Serial.print("OCR1A: ");
		Serial.println(OCR1A);
		t = micros();
		timer1_over = 0;
	}
	//taktermittlung();
}

int sensorFeuchtTempAbfrage(int pin) {
	//Zeitintervall der Abfrage ueberpruefen
	if ((millis() - letzteMesszeitpunkt) < abrufIntervallSekunden * 1000) {
		if (letzteMesszeitpunkt != 0) {
			return -99;  // nach Einschalten soll gemessen werden
		}
	}

	letzteMesszeitpunkt = millis();
	Serial.print("MILLIS ");
	Serial.println(millis()/1000);

	//Variablen
	int16_t fehler[6];
	int8_t i = 0, j = 0;	//Zaehlervariablen
	uint8_t wert[5];    //Bytes des Sensorwertes
	uint8_t bitwert = 7;  //Bit der einzelnen Bytes der Sensorwertes
	uint8_t sensorBitWert;  //Variable zum Festhalten des aktuellen Bits am Sensor
	uint16_t zaehler;
	uint16_t sum = 0;	//Paritaetssumme

	// BUFFER leeren
	for (int i = 0; i < 5; i++) {
		fehler[i] = 0;
		wert[i] = 0;
	}
	for (int i = 5; i < 10; i++){
		fehler[i] = 0;
	}

	
	//-------------------- Sensor starten - PIN als Ausgang verwenden (laut Datenblatt) ---------------------------
	timer2_over = 0;
	TCNT2 = 0; // setzte timer 0 zurueck
	DDRD |= (1 << pin);	// als Ausgang setzen
	PORTD &= ~(1 << pin);	// auf LOW setzen (invertiere den Port und setze mit Register PORTD zusammen
	zaehler = MAXZAEHL;
	//36*0,5ms = 18ms LOW
	while (timer2_over < 36){
		if (zaehler-- <= 0){
			fehler[0] = -timer2_over;
			break;
		}
		fehler[0] = timer2_over;
	}
	PORTD |= (1 << pin);	// auf HIGH setzen
	TCNT2 = 0; // setzte timer 0 zurueck
	timer2_over = 0;
	zaehler = MAXZAEHL;
	//1 count = 2 Mikro-Sekunden -> 40 Mikrosekunden
	while (TCNT2 < 20) {
		if (zaehler-- <= 0) {
			fehler[1] = -TCNT2;
			break;
		}
		fehler[1] = TCNT2;
	}
	DDRD &= ~(1 << pin);	// PIN als Eingang setzen

	cli();
	//Interrupt einstellen
	EIMSK |= (1 << INT1);
	EICRA |= (1 << ISC10);
	sei();

	// ------------------------- Antwort vom Sensor: 1. Bit ist LOW, 2. Bit ist HIGH ---------------------------------
	//EICRA |= (1 << ISC11); //fallende Flanke erzeugt einen Interrupt
	portInterruptPD3 = 0;
	zaehler = MAXZAEHL;
	TCNT2 = 0; // setzte timer 0 zurueck
	timer2_over = 0;	//Warte 4us
	while (TCNT2 <= 2){
		if (zaehler-- <= 0) {
			fehler[2] = -TCNT2;
			break;
		}
	}
	do {
		if (zaehler-- <= 0) {
			fehler[2] = -TCNT2;
			break;
		}
		fehler[2] = TCNT2;
	} while (portInterruptPD3 != 0);

	//EICRA |= (1 << ISC11) | (1 << ISC10); //steigende Flanke erzeugt einen Interrupt
	TCNT2 = 0; // setzte timer 0 zurueck
	timer2_over = 0;
	portInterruptPD3 = 0;
	zaehler = MAXZAEHL;
	//Warte 4us
	while (TCNT2 <= 2) {
		if (zaehler-- <= 0) {
			fehler[3] = -TCNT2;
			break;
		}
	}
	zaehler = MAXZAEHL;
	do {
		if (zaehler-- <= 0) {
			fehler[3] = -TCNT2;
			break;
		}
		fehler[3] = TCNT2;
	} while (portInterruptPD3 != 0);

	//-------------------------- 1.Pause zwischen den Bits: 50us ------------------------------------
	//Pausenzeit zwischen den Datenbits ist 50 Microsekunden
	//EICRA &= ~(1 << ISC10);
	//EICRA |= (1 << ISC11); //fallende Flanke erzeugt einen Interrupt
	portInterruptPD3 = 0;
	zaehler = MAXZAEHL;
	TCNT2 = 0; // setzte timer 0 zurueck
	timer2_over = 0;
	//Warte 4us
	while (TCNT2 <= 2) {
		if (zaehler-- <= 0) {
			fehler[4] = -TCNT2;
			break;
		}
	}
	do {
		if (zaehler-- <= 0) {
			fehler[4] = -TCNT2;
			break;
		}
		fehler[4] = TCNT2;
	} while (portInterruptPD3 != 0);


	//----------------------------------- ab jetzt kommen die Daten ---------------------------------------
	for (i = 0; i < 5; i++) {
		for (j = 7; j >= 0; j--) {
			//EICRA |= (1 << ISC11) | (1 << ISC10); //steigende Flanke erzeugt einen Interrupt
			portInterruptPD3 = 0;
			zaehler = 0;
			//Warte 4us
			while (TCNT2 <= 1) {
				if (zaehler++ >= 1000) {
					fehler[3] = -1;
					break;
				}
			}
			do {
				if (zaehler++ >= 1000) {
					fehler[3] = -5;
					break;
				}
			} while (portInterruptPD3 != 0);
			portInterruptPD3 = 0;

			//Datenbit
			//EICRA &= ~(1 << ISC10);
			//EICRA |= (1 << ISC11); //fallende Flanke erzeugt einen Interrupt
			TCNT2 = 0; // setzte timer 0 zurueck
			timer2_over = 0;
			zaehler = 0;
			//Warte 4us
			while (TCNT2 <= 1) {
				if (zaehler++ >= 1000) {
					fehler[3] = -1;
					break;
				}
			}
			do {
				if (zaehler++ >= 1000) {
					fehler[3] = -4;
					break;
				}
			} while (portInterruptPD3 != 0);
			portInterruptPD3 = 0;
			TCCR2B = 0;
			zaehler = TCNT2;
			
			//gesendetes Bit HIGH?
			if (zaehler >= 10) {	//4us pro hochgezaehlten Bit
				wert[i] |= (1 << j);
			}
		}
	}
	
	// +++++++++++++++++++
	delay(20);
	// ++++++++++++++++

	DDRD |= (1 << feuchtLuftPD);  // als Ausgang setzen

	
	  //Paritaetspruefung
	sum = wert[0] + wert[1] + wert[2] + wert[3];
	if (sum != wert[4]) {
		fehler[4] = -5;
	}

	fehler[5] = fehler[0] + fehler[1] + fehler[2] + fehler[4];

	if (sum != wert[4]) {
		Serial.println("Paritaetspruefung fehlgeschlagen");
	}
	// Anzeige zur Fehlereingrenzung)
	for (i = 0; i < 10; i++) {
		Serial.print("Fehler");
		Serial.print(i);
		Serial.print(" ");
		Serial.print(fehler[i]);
		Serial.println(", ");
	}
	Serial.print(", SensorBitWert: ");
	Serial.println(sensorBitWert);
	for (i = 0; i < 5; i++) {
		Serial.print("Bit ");
		Serial.print(i);
		Serial.print(" ");
		Serial.print(wert[i]);
		Serial.println(", ");
	}
	Serial.print("\nParitaetssumme: ");
	Serial.println(sum);
	Serial.println("\n\n");



	if (fehler[5] != 0) {
		return fehler[5];
	}

	wetterSensor[0] = micros();
	wetterSensor[1] = wert[2];
	wetterSensor[2] = wert[0];

	return 0;
}

// Start des Timers2
void timer2start() {
	TCNT2 = 0; // setzte timer 0 zurueck
	timer2_over = 0;
//	Serial.println("TIMER2 start");
}

void taktermittlung() {
	//TCCR2B |= (1 << CS21) | (1 << CS20);	//TIMER0 PRESCALER auf 64 einstellen
	//TCNT2 = 0; // setzte timer 0 zurueck
	while(1){
		TCNT2 = 0; // setzte timer 0 zurueck
		timer2_over = 0;
		while (TCNT2 <= 9) {}
		PORTB ^= (1 << ledgePD);
		//TCNT2 = 0;
	}
}