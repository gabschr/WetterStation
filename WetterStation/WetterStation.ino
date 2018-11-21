//Debug Modus
#define DEBUG 1 //auskommentieren für reale Nutzung

//Display - Bibliotheken
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Display auf Port initalisieren
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
const int lcdBreite = 20;
const int lcdHoehe = 2;

//Konstanten
#define MAXZAEHL 65000
#define abrufIntervallSekunden 30

// GLOBALE VARIABLEN
double wetterSensor[4]; //Array fuer Temp und Feuchtigkeit (0: Nr. des Sensors, 1: Messzeitpunkt der Messung (ab Start vom Arduino in s), 2.Wert: Feucht, 3.Wert: Temp
unsigned int eepromMaximum = 0;
uint8_t timerTaster = 0;
uint8_t blinkzeitLed = 0;	//zwischen 0 und 12, 0-> gar nicht blinken; 12 (ca.7s leuchten)- langsames blinken; 1-schnelles blinken

volatile uint8_t timer1_over = 0;
volatile uint8_t timer2_over = 0;
volatile uint8_t portInterruptPD3 = 0;

bool tasterVorher = 0;
bool tasterWert = 1, tasterVerl = 1;


// PORTDEKLARATIONEN
#define ledrtPD	PB5			//PIN fuer rote LED
#define ledgePD	PB4			//PIN fuer geldbe 
#define taster  PB0			//PIN fuer Taster zum Umschalten der Sensoren (PIN8)
#define feuchtLuftPD  PD3	//PIN fuer den Feuchtigkeits- und Temp- Sensor (PIN3)
#define dht22Sensor   PD4	//PIN fuer DHT22-Sensor (Feuchtigkeit+Temp)



// ######################## INTERRUPT SERVICE ROUTINEN ###############################

// Routine zum Abfragen der Taster
ISR(PCINT0_vect) {
	byte portWert = 0;
	uint8_t aktuellerTasterWert;

	portWert = PINB;	//welcher Port wurde angesprochen?
	// wurde Taster gedrueckt?
	aktuellerTasterWert = (portWert & (1 << taster) >> taster);
	if (aktuellerTasterWert == 0) {	//wurde taster angesprochen?
		tasterWert = (PINB & (1 << taster)) >> taster;
#ifdef DEBUG
		//Serial.print("Interrupt Taster, Tasterwert, tasterVorher: ");
		//Serial.println(tasterWert);
		//Serial.println(tasterVorher);
#endif
		//wurde gedrueckt? (nicht los gelassen?)
		if (tasterWert == 0){
			// entprellen
			if (tasterVorher == 0) {	// Taster wurde zum ersten Mal gedrueckt
				tasterVorher = 1;
				timerTaster = 0;
				return;
			}
		}
	}
}

// Routine zum Abfragen von Sensoren nicht am Port D2 und D3
//ISR(PCINT2_vec) { //Sensoren

//}

// Abfragen des Sensors am Port D3 (DHT11)
ISR(INT1_vect)
{
	portInterruptPD3++;
}

// Timer1 Taster und rote LED
ISR(TIMER1_COMPA_vect) {
	uint8_t timerteiler = 0;

	if (tasterVorher > 0) {
		// 30 ms nach Tastendruck warten, um Prellen auszuschließen
		if (timerTaster > 0) {
			PORTB ^= (1 << ledgePD);  // LED toggelnd
			tasterVorher = 0;
		}
	}
	if (blinkzeitLed > 0) {
		if (timer1_over/20 == blinkzeitLed) {
			PORTB ^= (1 << ledrtPD);  // LED toggelnd
			timer1_over = 0;
		}
	}
	timer1_over++;
	timerTaster++;
}

// Timer2 (Taster)
ISR(TIMER2_COMPA_vect) {
	timer2_over++;
	//Serial.print("TIMER2: ");
	//Serial.println(timer2_over);
}


// ############################ SETUP-ROUTINE ########################################
void setup() {
	lcd.begin(lcdBreite, lcdHoehe);
#ifdef DEBUG
	// Ausgabe am Monitor vorbereiten (Debug)
	Serial.begin(9600);
	Serial.println("\n\nAusgabe am Monitor");
	Serial.println("------------------");
	Serial.print("eepromMaximum = ");
	Serial.println(eepromMaximum);
	Serial.println("------------------");
#endif

	//LCD-Display aktivieren
	lcd.begin(20, 4);

	// Deklarieren der I-O-Ports und Setzen der Pegel
	DDRB |= (1 << ledrtPD);     // als Ausgang
	PORTB |= (1 << ledrtPD);	//HIGH setzen
	DDRB |= (1 << ledgePD);		// als Ausgang
	PORTB |= (1 << ledgePD);	//HIGH setzen

	DDRB &= ~(1 << taster); // als Eingang setzen - LOW-aktiv 
	PORTB |= (1 << taster); // Pull-Up-Widerstand am Arduino setzen
	DDRD |= (1 << feuchtLuftPD);  // als Ausgang setzen
	DDRD |= (1 << dht22Sensor); // als Ausgang setzen

	// leere Array wetterSensor
	for (int i = 0; i < 5; i++) {
		wetterSensor[i] = 0;
	}

	// Array mit Sensor-Nummern befüllen
	wetterSensor[0] = 11;

	// ()()()()()())()()()()() INTERRUPTS ()()()()()()()()()()()()()()()()()()()()
	cli();	//deaktivieren von Interrupts

	// TIMER1 fuer Taster und LED
	TCCR1A = 0; // TCCR1A register auf 0 setzen
	TCCR1B = 0; // TCCR1B register auf 0 setzen
	TCNT1 = 0; // Zaehlerwert zuruecksetzen
	OCR1A = 7499;	//MAX bei 16-BIT-TIMER: 65535, bei 8-Bit 2 hoch 8 - 1 = 255
	TCCR1B |= (1 << CS11) | (1 << CS10);
	TCCR1B |= (1 << WGM12); // CTC Mode
	TIMSK1 |= (1 << OCIE1A); // timer compare interrupt aktivieren

	// TIMER2 fuer Sensorabfrage
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;
	OCR2A = 124;
	TCCR2B |= (1 << CS21) | (1 << CS20);
	TCCR2B |= (1 << WGM22);
	TIMSK2 |= (1 << OCIE2A);

	// -------------- EXTERNAL INTERRUPTS ----------------------------
	// Interrupt auf PORTB-Gruppe (Taster)
	PCICR |= (1 << PCIE0);		// PIN CHANGE INTERRUPT FUER PB-Gruppe (TASTER)
	PCMSK0 |= (1 << PCINT0);	//Externer Interruppt fuer Port PD2

	//// Interrupt auf PORTD-Gruppe (Sensoren)
	//PCICR |= (1 << PCIE2);
	//PCMSK2 |= (1 << PCINT2);

	// Interrupt auf PORT PD3 (Sensor DHT11)
	EICRA |= (1 << ISC10); //fallende und steigende Flanke erzeugt einen Interrupt
	EIMSK |= (1 << INT1);

	sei();	//aktivieren von Interrupts
}

// ########################## HAUPTPROGRAMM (LOOP) ######################################
void loop() {
	int16_t fehler = 0;
	fehler = sensorFeuchtTempAbfrage(feuchtLuftPD, 11);
#ifdef DEBUG
	if (fehler == 0) {
		// Anzeige auf Seriellen Monitor
		Serial.print("Sensor-Daten:\nZeit: ");
		Serial.print(wetterSensor[1]);
		Serial.print("s, Temperatur: ");
		Serial.print(wetterSensor[2]);
		Serial.print(", Feuchtigkeit: ");
		Serial.print(wetterSensor[3]);
#endif

		// Anzeige auf Display
		lcdAnzeige();
	}
}

// ############## ABFRAGEN DES DIGITALEN SENSORS ############################
int sensorFeuchtTempAbfrage(uint8_t pin, uint8_t sensorNr) {
	//Zeitintervall der Abfrage ueberpruefen
	if ((millis()/1000 - wetterSensor[1]) < abrufIntervallSekunden) {
		if (wetterSensor[1] != 0) {
			return -99;  // nach Einschalten soll gemessen werden
		}
	}
	wetterSensor[1] = millis() / 1000;

	//Variablen
	int16_t kontrolle[10];
	int8_t i = 0, j = 0;	//Zaehlervariablen
	uint8_t wert[5];    //Bytes des Sensorwertes
	uint8_t bitwert = 7;  //Bit der einzelnen Bytes der Sensorwertes
	uint16_t zaehler;
	uint16_t sum = 0;	//Paritaetssumme

	// BUFFER leeren
	for (i = 0; i < 5; i++) {
		wert[i] = 0;
	}
	for (i = 0; i < 10; i++) {
		kontrolle[i] = 0;
	}

	//-------------------- Sensor starten - PIN als Ausgang verwenden (laut Datenblatt) ---------------------------
	timer2_over = 0;
	TCNT2 = 0; // setzte timer 0 zurueck
	DDRD |= (1 << pin);	// als Ausgang setzen
	PORTD &= ~(1 << pin);	// auf LOW setzen
	zaehler = MAXZAEHL;

	//36*0,5ms (Zaehldauer Timer2) = 18ms LOW
	while (timer2_over < 36) {
		if (zaehler-- <= 0) {
			kontrolle[0] = -timer2_over;
			break;
		}
		kontrolle[0] = timer2_over;
	}
	TCNT2 = 0;
	PORTD |= (1 << pin);	// auf HIGH setzen
	timer2_over = 0;
	zaehler = MAXZAEHL;
	//1 count = 2 Mikro-Sekunden -> 40 Mikrosekunden
	while (TCNT2 < 22) {
		if (zaehler-- <= 0) {
			kontrolle[1] = -TCNT2;
			break;
		}
		kontrolle[1] = TCNT2;
	}
	DDRD &= ~(1 << pin);	// PIN als Eingang setzen

	cli();
	//Interrupt einstellen
	EIMSK |= (1 << INT1);
	EICRA |= (1 << ISC10);
	sei();

	// ------------------------- Antwort vom Sensor: 1. Bit ist LOW, 2. Bit ist HIGH, 3. Bit LOW: PAUSE -------------------------------
	for (i = 2; i < 4; i++) {
		TCNT2 = 0; // setzte timer 0 zurueck
		timer2_over = 0;
		portInterruptPD3 = 0;
		zaehler = MAXZAEHL;

		do {
			if (zaehler-- <= 0) {
				kontrolle[i] = -TCNT2;
				break;
			}
			kontrolle[i] = TCNT2;
		} while (portInterruptPD3 == 0);
	}


	//----------------------------------- ab jetzt kommen die Daten ---------------------------------------
	// 5 BYTES Daten
	for (i = 0; i < 5; ++i) {
		// 8 Bits pro Byte Daten (beginn mit MSB)
		for (j = 7; j >= 0; j--) {
			//PAUSE, danach DATEN:
			for (int k = 5; k < 7; k++) {
				portInterruptPD3 = 0;
				zaehler = MAXZAEHL;
				TCNT2 = 0; // setzte timer 0 zurueck
				timer2_over = 0;
				do {
					if (zaehler-- <= 0) {
						kontrolle[k] = -TCNT2;
						break;
					}
					kontrolle[k] = TCNT2;
				} while (portInterruptPD3 == 0);
				portInterruptPD3 = 0;
			}
			sum = TCNT2;
			portInterruptPD3 = 0;

			//gesendetes Bit HIGH?
			if (sum >= 20) {	//4us pro hochgezaehlten Bit
				wert[i] |= (1 << j);
			}
			kontrolle[7] = sum;
		}
	}
	DDRD |= (1 << feuchtLuftPD);  // als Ausgang setzen
	//Paritaetspruefung
	sum = wert[0] + wert[1] + wert[2] + wert[3];
	if (sum != wert[4]) {
		kontrolle[8] = -5;
	}

#ifdef DEBUG
	if (sum != wert[4]) {
		Serial.println("Paritaetspruefung fehlgeschlagen");
	}
	// Anzeige zur Fehlereingrenzung)
	Serial.println();
	for (i = 0; i < 10; i++) {
		Serial.print("Fehler");
		Serial.print(i);
		Serial.print(" ");
		Serial.print(kontrolle[i]);
		Serial.println(", ");
	}
	Serial.println("\nBitwerte:\n------------");
	for (i = 0; i < 5; i++) {
		Serial.print("Bit");
		Serial.print(i);
		Serial.print(" - ");
		Serial.print(wert[i]);
		Serial.println(", ");
	}
	Serial.print("\nParitaetssumme: ");
	Serial.println(sum);
	Serial.println("\n\n");
#endif

	for (i = 1; i < 10; i++) {
		if (kontrolle[i] > 0) {
			kontrolle[i] = 0;
		}
		kontrolle[9] += kontrolle[i];
	}

	if (kontrolle[9] != 0) {
		return kontrolle[9];
	}

	//Werte schreiben in Array
	wetterSensor[1] = millis() / 1000;
	wetterSensor[2] = wert[3];
	wetterSensor[2] = wetterSensor[2]/10+ wert[2];
	wetterSensor[3] = wert[0];

	Serial.println("WetterSensor: ");
	for (i = 0; i < 5; i++) {
		Serial.print(i);
		Serial.print(":");
		Serial.print(wetterSensor[i]);
		Serial.println();
	}
	Serial.println();

	return 0;
}

void lcdAnzeige() {
	lcd.clear();

	//Werte auf LCD drucken
	lcd.setCursor(0, 0);
	lcd.print("Luftf.: ");
	lcd.setCursor(10, 0);
	lcd.print(wetterSensor[3]);
	lcd.setCursor(0, 1);
	lcd.print("Temp.: ");
	lcd.setCursor(10, 1);
	lcd.print(wetterSensor[2]);
}
