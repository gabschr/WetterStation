/***************************************************************
 *
 * Projekt:     Wetterstation mit Innen / Außentemperatur
 * Version:     V0.1
 * Date:        01.12.2012
 * Author:      Gabriel Schreyer, Henning Nußbaum
 * Company:     BA Dresden
 *
****************************************************************/

//************* Debug Modus ************************************
#define DEBUG 1 //auskommentieren für reale Nutzung

//************* Includes ***************************************
//Display - Bibliotheken
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define MAXZAEHL 65000
#define abrufIntervallSekunden 5
#define abstandHistorie 15
// PORTDEKLARATIONEN
#define ledrtPD  PB5     //PIN fuer rote LED
#define ledgePD PB4     //PIN fuer gelbe LED
#define taster  PB0     //PIN fuer Taster zum Umschalten der Sensoren (PIN8)
#define feuchtLuftPD  PD3 //PIN fuer den Feuchtigkeits- und Temp- Sensor (DHT11) (PIN3)
#define dht22Sensor   PD4 //PIN fuer DHT22-Sensor (Feuchtigkeit+Temp) (PIN4)
#define analogFeucht  A0  //Analog-Feuchtigkeit-Sensor (PORT A0 - 14)
#define analogTemp    A1  //Analog-Temperatur-Sensor (PORT A1 - 15)

//************* globale Variablen *******************************
const int lcdBreite = 16; //breite des Displays
const int lcdHoehe = 2; //Höhe des Displays
const uint8_t historischeWerteAnzahl = 5; //Anzahl gespeicherter Messergebnisse
const uint8_t sensorAnzahl = 3; // Anzahl vorhandener Sensoren
//Array fuer Temp und Feuchtigkeit, v.l.n.r.: Historie- 3.Dimension,  Sensor-Nr.(PORT)- 2. Dimension, Sensor-Werte- 1. Dimension
//Werte 1.Dimesion: 0: Nr. des Sensors, 1: Messzeitpunkt der Messung (ab Start vom Arduino in s), 2.Wert: Temp, 3.Wert: Luftfeuchte
double wetterSensor[historischeWerteAnzahl][sensorAnzahl + 1][4];

uint32_t letzteAbrufzeit[sensorAnzahl + 1][2]; //0. Stelle: Senssor-Typ, 1.Stelle: letzter Abfragewert Sensor
uint32_t letzteVerschiebungszeitpunkt;

volatile uint8_t timer0_over = 0;
volatile uint32_t timer1Sek = 0;	//fortwaehrende Sekundenzaehlung
volatile uint8_t timer2_over = 0;
volatile uint8_t portInterrupt = 0;

uint8_t timerTaster = 0;
uint8_t blinkzeitLedRt = 0;	//zwischen 0 und 15, 0-> gar nicht blinken; 12- langsames blinken; 1-schnelles blinken
uint8_t blinkzeitLedGe = 0;
int8_t tasterBetaetigung = 0;	//0: nicht gedruckt, -1: muss entprellt werden, -2: ist entprellt, 1: gedrueckt, 2: lang gedruckt
uint8_t anzeigeSensor = 0;	//welcher Sensor soll angezeigt werden? (0 -> 0.Wert im Array)
String sensorBezeichnung = "unbk.";

//************* eigene Symbole ********************************
char thermometerChar = 0;
byte thermometerByte[8] = { B01110, B01010, B01110, B01110, B11111, B11111, B11111, B01110 };
char gradChar = 1;
byte gradZeichen[8] = { B11100, B10100, B11100, B00000, B00000, B00000, B00000, B00000 };
char celsiusChar = 2;
byte clesiusZeichen[8] = { B01110,B10001,B10000,B10000,B10000,B10000,B10001,B01110 };
char prozentChar = 3;
byte prozentZeichen[8] = { B11000,B11000,B00001,B00010,B00100,B01000,B10011,B00011 };
char tropfenChar = 4;
byte tropfenZeichen[8] = { B00100,B00100,B01010,B01010,B10001,B10001,B10001,B01110 };
char deltaChar = 5;
byte deltaZeichen[8] = { B00000, B00000, B00000, B00100, B01010, B11111, B00000, B00000 };

//Display auf Port initalisieren
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

//************* INTERRUPT SERVICE ROUTINEN *********************
// Routine zum Abfragen der Taster
ISR(PCINT0_vect) {
	uint8_t aktuellerTasterWert;

	// wurde Taster gedrueckt? // LOW-aktiv
	aktuellerTasterWert = (PINB & (1 << taster) >> taster);
	//wurde taster zum ersten Mal oder waehrend Testroutne betaetigt?
	if (aktuellerTasterWert == 0 && tasterBetaetigung == 0) {
		tasterBetaetigung = -1;
		timerTaster = 0;
		PORTB |= (1 << ledgePD);  //gelbe LED als Kontrolle an
		return;
	}
	if ((aktuellerTasterWert == 1) && (tasterBetaetigung == -2)) {	//Taster wurde losgelassen und war vorher stabil
		PORTB &= ~(1 << ledgePD); //gelbe LED aus
		if (timerTaster > 200) {	//(Testmodus)
			tasterBetaetigung = 99;
			return;
		}
		if (timerTaster > 100) {	//2s
			tasterBetaetigung = 2;
			return;
		}
		tasterBetaetigung = 1;
		return;
	}
	if (aktuellerTasterWert == 0 && tasterBetaetigung >= 2) {	//Abbruch der Testroutine
		tasterBetaetigung = 0;
		anzeigeSensor = 0;
		wetterSensor[0][sensorAnzahl][2] = 0;
		wetterSensor[0][sensorAnzahl][3] = 0;
		anzeigeSensor = 0;
		PORTB &= ~(1 << ledgePD);	//LEDge ausschalten
	}
	return;
}

// Abfragen des Sensors am Port D3 (DHT11)
ISR(INT1_vect)
{
	portInterrupt = 3;
}

// Routine zum Abfragen von Sensoren nicht am Port D2 und D3
ISR(PCINT2_vect) {
	//PCINT20 -> fuer Port D4 zustaendig
	if ((PIND & (1 << PCINT20) >> PCINT20) == 1) {
		portInterrupt = 4;
	}
	return;
}

// Timer2 Taster und rote LED (16ms)
ISR(TIMER0_COMPA_vect) {
	timer0_over++;
	timerTaster++;
	blinkzeitLedGe++;
	// 50 ms nach Tastendruck warten, um Prellen auszuschließen
	if (tasterBetaetigung < 0 && timerTaster > 3) {
		tasterBetaetigung = -2;
	}
	// wenn waehrend des Entprellens Taster los gelassen wurde, soll LED auch irgendwann aus gehen
	// Signalisierung, wann Taster so lange gedrueckt wurde, dass Testmodus anspringt
	if (timerTaster > 110) {
		PORTB &= ~(1 << ledgePD); //gelbe LED aus
		if (timerTaster > 190 && timerTaster < 210 && tasterBetaetigung != 0) {
			PORTB |= (1 << ledgePD);
			if (timerTaster > 380) {
				PORTB &= ~(1 << ledgePD);
			}
		}
	}
	if ((blinkzeitLedRt > 0) && ((timer0_over / 6) == blinkzeitLedRt)) {
		PORTB ^= (1 << ledrtPD);  //LED toggelnd
		timer0_over = 0;
	}
	if (tasterBetaetigung == 99 && blinkzeitLedGe > 50) {	//TEST-MODUS?
		PORTB ^= (1 << ledgePD);  //LED toggelnd
		blinkzeitLedGe = 0;
	}
	return;
}

// Timer1 (Sekunden fortlaufend)
ISR(TIMER1_COMPA_vect) {
	timer1Sek++;
	return;
}

// Timer0 (Sensoren)
ISR(TIMER2_COMPA_vect) {
	timer2_over++;
	return;
}


//************* SETUP-ROUTINE **********************************
void setup() {
	//Display initialisieren
	lcd.begin(lcdBreite, lcdHoehe);
	lcd.createChar(thermometerChar, thermometerByte);
	lcd.createChar(gradChar, gradZeichen);
	lcd.createChar(celsiusChar, clesiusZeichen);
	lcd.createChar(prozentChar, prozentZeichen);
	lcd.createChar(tropfenChar, tropfenZeichen);
	lcd.createChar(deltaChar, deltaZeichen);
#ifdef DEBUG
	// Ausgabe am Monitor vorbereiten (Debug)
	Serial.begin(9600);
	Serial.println("\n\nAusgabe am Monitor");
#endif

	// Deklarieren der I-O-Ports und Setzen der Pegel
	DDRB |= (1 << ledrtPD);     // als Ausgang
	PORTB &= ~(1 << ledrtPD);	//LOW setzen
	DDRB |= (1 << ledgePD);		// als Ausgang
	PORTB &= ~(1 << ledgePD);	//LOW setzen

	DDRB &= ~(1 << taster); // als Eingang setzen - LOW-aktiv 
	PORTB |= (1 << taster); // Pull-Up-Widerstand am Arduino setzen
	DDRD |= (1 << feuchtLuftPD);  // als Ausgang setzen
	DDRD |= (1 << dht22Sensor); // als Ausgang setzen

	// Variablen zuruecksetzen
	// leere Array wetterSensor und fülle mit Sensor-Nummer
	for (uint8_t i = 0; i < historischeWerteAnzahl; i++) {
		for (uint8_t j = 0; j < sensorAnzahl + 1; j++) {
			for (uint8_t k = 0; k < 4; k++) {
				wetterSensor[i][j][k] = 0;
			}
		}
		//Sensordaten (PORTS) fuellen
		wetterSensor[i][0][0] = feuchtLuftPD;
		wetterSensor[i][1][0] = dht22Sensor;
		wetterSensor[i][2][0] = analogFeucht;
		wetterSensor[i][sensorAnzahl][0] = 99;	//TESTSENSORWERT
	}

	// leere Array letzte Zugriffszeit und setze Sensor-Nr.
	for (uint8_t i = 0; i < sensorAnzahl; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			letzteAbrufzeit[i][j] = 0;
		}
	}
	letzteAbrufzeit[0][0] = 11;	//SENSOR-TYP DHT11
	letzteAbrufzeit[1][0] = 22; //SENSOR-TYP DHT22
	letzteAbrufzeit[2][0] = 1;	//SENSOR-TYP ANALOG AMT1001 + LM35 fuer Temp.
#ifdef DEBUG
	for (uint8_t i = 0; i < sensorAnzahl + 1; i++) {
		Serial.print("SENSORWERT");
		Serial.print(i);
		Serial.print(": ");
		Serial.println(wetterSensor[0][i][0]);
		Serial.print("letzteAbrufzeit");
		Serial.print(i);
		Serial.print(": ");
		Serial.println(letzteAbrufzeit[i][0]);
	}
#endif // DEBUG

	// ADC einstellen
	// RESET der Register
	ADMUX = 0;
	ADCSRA = 0;
	ADCSRB = 0;

	ADMUX |= (1 << REFS0);	//Referenz 5V
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1); //ADC prescaler auf 64 => ADC Freq. = 250kHz
	ADCSRB |= (1 << ADTS0); //ANALOG COMPARATOR

	// ()()()()()())()()()()() INTERRUPTS ()()()()()()()()()()()()()()()()()()()()
	cli();	//deaktivieren von Interrupts

	// TIMER0 fuer Taster und LED
	TCCR0A = 0;
	TCCR0B = 0;
	TCNT0 = 0;
	OCR0A = 255;	//MAX bei 16-BIT-TIMER: 65535, bei 8-Bit 2 hoch 8 - 1 = 255
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR1B |= (1 << WGM12); // CTC Mode
	TIMSK0 |= (1 << OCIE1A); // timer compare interrupt aktivieren

	// TIMER1 fuer fortlaufende Sek-Zaehlung
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	OCR1A = 15624;
	TCCR1B |= (1 << CS12) | (1 << CS10);
	TCCR1B |= (1 << WGM22);
	TIMSK1 |= (1 << OCIE2A);

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
	PCICR |= (1 << PCIE0);		//PIN CHANGE INTERRUPT FUER PB-Gruppe (TASTER)
	PCMSK0 |= (1 << PCINT0);	//Maske fuer Port PB8 (TASTER)

	// Interrupt auf PORTD-Gruppe (Sensoren)
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT20);	//PORT PD4

	// Interrupt auf PORT PD3 (Sensor DHT11)
	EICRA |= (1 << ISC10); //fallende und steigende Flanke erzeugt einen Interrupt
	EIMSK |= (1 << INT1);

	sei();	//aktivieren von Interrupts
}


//************* HAUPTPROGRAMM (LOOP) ***************************
void loop() {
	int8_t kontrolle = 0;
	kontrolle = sensorFeuchtTempAbfrage(feuchtLuftPD);

	// Events bei neuen Sensorwerten
	if (kontrolle > 0) {
		tasterAuswerten();
	}
#ifdef DEBUG
	if (kontrolle > 0) {
		for (uint8_t i = 0; i < 3; i++) {
			// Anzeige auf Seriellen Monitor
			Serial.print("\nSensor-Nummer: ");
			Serial.print(wetterSensor[0][i][0]);
			Serial.print(" Sensor-Daten:\nZeit: ");
			Serial.print(wetterSensor[0][i][1]);
			Serial.print("s, Temperatur: ");
			Serial.print(wetterSensor[0][i][2]);
			Serial.print(", Feuchtigkeit: ");
			Serial.println(wetterSensor[0][i][3]);
			Serial.print("letzte Abrufzeit: ");
			Serial.println(letzteAbrufzeit[i][1]);
			Serial.print("Kontrolle: ");
			Serial.println(kontrolle);
		}
	}
#endif
	kontrolle = sensorFeuchtTempAbfrage(dht22Sensor);

#ifdef DEBUG
	if (kontrolle != -99) {
		Serial.print("Kontrolle: ");
		Serial.println(kontrolle);
		Serial.print("LED Blinkzeit: ");
		Serial.println(blinkzeitLedRt);
		Serial.print("***************************\nTasterBetaetigung: ");
		Serial.println(tasterBetaetigung);
		Serial.print("Anzeige Sensor: ");
		Serial.println(anzeigeSensor);
	}
#endif // DEBUG

	// Events bei neuen Sensorwerten
	if (kontrolle > 0) {
		tasterAuswerten();
	}

	kontrolle = analogeSensoren(analogFeucht);
	if (kontrolle > 0) {
		tasterAuswerten();
	}
	pruefungFeuchtigkeitUeber60();


	verlaufArrayVorschieben();

	if (tasterBetaetigung > 0) {
		tasterAuswerten();
	}

}

void tasterAuswerten() {
	// Events bei Tastendruck
	switch (tasterBetaetigung) {
	case 0:
		aktuelleWerteAnzeigen(lcdBreite, lcdHoehe);
		break;
	case 1:
		tasterBetaetigung = 0;
		anzeigeSensor = (anzeigeSensor + 1) % (sensorAnzahl);
		aktuelleWerteAnzeigen(lcdBreite, lcdHoehe);
		break;
	case 2:
	case 3:
		wertAenderungAnzeigen();
		break;
	case 99:
		testWetterStation();
		break;
	default:
		break;
	}
}

void wertAenderungAnzeigen() {
	//double abrufDifferenz = timer1Sek - letzteAbrufzeit[anzeigeSensor][2];
	double wertAenderungProzentual = wetterSensor[0][anzeigeSensor][3] - wetterSensor[1][anzeigeSensor][3];
	if (letzteVerschiebungszeitpunkt == timer1Sek || tasterBetaetigung == 2) {
		prozentBalkenZeigen(wertAenderungProzentual, lcdBreite);
	}
	tasterBetaetigung = 3;
}

//************* ABFRAGEN DES DIGITALEN SENSORS *****************
int sensorFeuchtTempAbfrage(uint8_t pin) {
	//Variablen
	int8_t sensorImArray = -1;
	int8_t i = 0, j = 0;	//Zaehlervariablen
	uint8_t wert[5];    //Bytes des Sensorwertes
	uint8_t bitwert = 7;  //Bit der einzelnen Bytes des Sensorwertes
	uint16_t sum = 0;	//Zeit fuer Datenbit, Paritaetssumme
#ifdef DEBUG
	uint8_t probe[5];
#endif // DEBUG


	//Sensor-PIN im Array finden
	for (i = 0; i < sensorAnzahl; i++) {
		if (wetterSensor[0][i][0] == pin) {
			sensorImArray = i;
			break;
		}
	}
	if (sensorImArray < 0) {
		return -98;
	}

	//Zeitintervall der Abfrage ueberpruefen
	if (((timer1Sek)-letzteAbrufzeit[sensorImArray][1]) < abrufIntervallSekunden) {
		if (letzteAbrufzeit[sensorImArray][1] != 0) {// nach Einschalten soll gemessen werden
			return -99;
		}
	}
	letzteAbrufzeit[sensorImArray][1] = timer1Sek;

	// BUFFER leeren
	for (i = 0; i < 5; i++) {
		wert[i] = 0;
	}

	//--------------- Sensor starten - PIN als Ausgang verwenden und LOW- HIGH- Signal geben (laut Datenblatt) ---------------------------
	timer2_over = 0;
	TCNT2 = 0;				//setzte timer 0 zurueck
	DDRD |= (1 << pin);		//als Ausgang setzen
	PORTD &= ~(1 << pin);	//auf LOW setzen
	sum = MAXZAEHL;

	//DHT11: 18ms LOW, DHT22: 1ms LOW 
	switch (letzteAbrufzeit[sensorImArray][0]) {
	case 11: j = 36; break;	//DHT11
	case 22: j = 2; break;	//DHT22
	default: break;
	}
	while (timer2_over < j) {
		if (sum-- <= 0) {
			return -10;
		}
	}

	TCNT2 = 0;
	PORTD |= (1 << pin);	// auf HIGH setzen
	timer2_over = 0;
	sum = MAXZAEHL;
	//1 count = 2 Mikro-Sekunden -> 40 Mikrosekunden auf HIGH setzen
	while (TCNT2 < 22) {
		if (sum-- <= 0) {
			return -11;
		}
	}
	DDRD &= ~(1 << pin);	// PIN als Eingang setzen

	// ------------------------- Antwort vom Sensor: 1. Bit ist LOW, 2. Bit ist HIGH, 3. Bit LOW: PAUSE -------------------------------
	for (i = 2; i < 4; i++) {
		TCNT2 = 0;
		timer2_over = 0;
		portInterrupt = 0;
		sum = MAXZAEHL;
		//Warte 4us
		while (TCNT2 <= 2) {
			if (sum-- <= 0) {
				return -29 - i;
			}
		}
		do {
			if (sum-- <= 0) {
				return -20 - i;
			}
		} while (portInterrupt == 0);
	}

	//----------------------------------- ab jetzt kommen die Daten ---------------------------------------
	// 5 BYTES Daten
	for (i = 0; i < 5; ++i) {
		// 8 Bits pro Byte Daten (beginn mit MSB)
		for (j = 7; j >= 0; j--) {
			//PAUSE, danach DATEN
			for (int k = 5; k < 7; k++) {
				portInterrupt = 0;
				sum = MAXZAEHL;
				TCNT2 = 0; // setzte timer 0 zurueck
				timer2_over = 0;
				//Warte 4us
				while (TCNT2 <= 2) {
					if (sum-- <= 0) {
						return -29 - i;
					}
				}
				do {
					if (sum-- <= 0) {
						return -30 - k;
					}
				} while (portInterrupt == 0);
				portInterrupt = 0;
			}
			sum = TCNT2;
			portInterrupt = 0;

			//gesendetes Bit HIGH?
			if (sum >= 35) {	//2us pro hochgezaehlten Bit
				wert[i] |= (1 << j);
			}
#ifdef DEBUG
			probe[i] += sum;
#endif
		}
	}
	DDRD |= (1 << pin);  // als Ausgang setzen
	portInterrupt = sum;

	//Paritaetspruefung
	sum = wert[0] + wert[1] + wert[2] + wert[3];

	// Paritaetssumme im Falle von DHT22 bestimmen
	if (letzteAbrufzeit[sensorImArray][0] == 22) {
		sum = sum % 256;
	}

#ifdef DEBUG
	Serial.println("\n#############################");
	Serial.print("Sensor ");
	Serial.println(sensorImArray);
	if (sum != wert[4]) {
		Serial.println("Paritaetspruefung fehlgeschlagen");
	}
	Serial.println("\nBitwerte:\n------------");
	for (i = 0; i < 5; i++) {
		Serial.print("Bit");
		Serial.print(i);
		Serial.print(" - ");
		Serial.print(wert[i]);
		Serial.println(", ");
	}
	Serial.println("\nProbewerte:\n------------");
	for (i = 0; i < 5; i++) {
		Serial.print("Bit");
		Serial.print(i);
		Serial.print(" - ");
		Serial.print(probe[i] / 8);
		Serial.println(", ");
	}
	Serial.print("\nParitaetssumme: ");
	Serial.println(sum);
	Serial.print("Timer0: ");
	Serial.println(timer2_over);
	Serial.print("Port Interrupt: ");
	Serial.println(portInterrupt);
#endif

	if (sum != wert[4]) {
		return -50;
	}

	// Pruefung auf Plausibilitaet der Daten ->-> muss noch gemaccht werden

	//Werte schreiben in Array
	wetterSensor[0][sensorImArray][1] = timer1Sek;

	//unterschiedliche Auswertuung der Bits zwischen DHT11 und DHT22:
	//DHT22: schreibt 8 Bit-Werte, DHT22 schreibt 16-Bit-Werte, weiteres siehe Datenblatt
	switch (letzteAbrufzeit[sensorImArray][0]) {
	case 11:
		wetterSensor[0][sensorImArray][2] = (double)wert[3] / 10 + wert[2];
		wetterSensor[0][sensorImArray][3] = (double)wert[1] / 10 + wert[0];
		break;
	case 22:
		wetterSensor[0][sensorImArray][2] = (wert[2] % 128) * 256;
		wetterSensor[0][sensorImArray][2] = (wetterSensor[0][sensorImArray][2] + wert[3]) / 10;
		wetterSensor[0][sensorImArray][3] = wert[0] * 256;
		wetterSensor[0][sensorImArray][3] = (wetterSensor[0][sensorImArray][3] + wert[1]) / 10;
		if ((wert[2] & (1 << 7)) == 128) {
			wetterSensor[0][sensorImArray][2] *= -1;
		}
		break;
	default:
		return -97;
	}
	letzteAbrufzeit[sensorImArray][1] = wetterSensor[0][sensorImArray][1];
	return 1;
}

//************* ABFRAGEN DES ANALOGEN SENSORS ******************
// PIN, was uebergeben wird: Feuchtigkeitssensor, Temp-Sensor ein PIN danach
int analogeSensoren(int pin) {
	uint16_t zaehler = MAXZAEHL;
	uint16_t wert[2];
	int8_t sensorImArray = -1;

	for (uint8_t i = 0; i < sensorAnzahl; i++) {
		if (wetterSensor[0][i][0] == pin) {
			sensorImArray = i;
			break;
		}
	}
	if (sensorImArray < 0) {
		return -98;
	}
	pin -= 14; //fuer ADMUX-Register ANALOG-Port von 0 an gezaehlt

	//Werte zuruecksetzen
	wert[0] = wert[1] = 0;

	//Zeitintervall der Abfrage ueberpruefen
	if ((timer1Sek - letzteAbrufzeit[sensorImArray][1]) < abrufIntervallSekunden) {
		if (letzteAbrufzeit[sensorImArray][1] != 0) {// nach Einschalten soll gemessen werden
			return -99;
		}
	}
	letzteAbrufzeit[sensorImArray][1] = timer1Sek;

	ADCSRA |= (1 << ADEN); //ADC einschalten

	for (uint8_t sensor = 0; sensor < 2; sensor++) {
		zaehler = MAXZAEHL;
		ADMUX = (ADMUX & ~(0x1F)) | (pin & 0x1F); //KANAL waehlen ohne andere BITS zu beeinflussen
		for (uint8_t versuche = 0; versuche < 2; versuche++) {
			TCNT2 = 0;
			timer2_over = 0;
			zaehler = MAXZAEHL;
			ADCSRA |= (1 << ADSC);	//Wandlung starten
			while (ADCSRA & (1 << ADSC)) {
				if (zaehler-- <= 0) {
					return -20;
				}
			}
			wert[sensor] = ADCL;
			wert[sensor] += (ADCH << 8);
		}
		pin++;
	}
	ADCSRA &= ~(1 << ADEN); //ADC ausschalten

#ifdef DEBUG
	Serial.print("++++++++++++++ ANALOGER SENSOR +++++++++++++++++\nWERT0: ");
	Serial.println(wert[0]);
	Serial.print("WERT1: ");
	Serial.println(wert[1]);
#endif // DEBUG

	//Werte außerhalb des Messbereichs fuer Sensoren
	//Wert ueber ca. 3V
	if (wert[0] > 620) {
		return -90;
	}
	//Wert ueber ca. 1,5V
	if (wert[1] > 310) {
		return -90;
	}

	// Plausibilitaetspruefung muss noch durchgeführt werden

	//Werte ausgeben- DIGITALWERT richtig umrechnen
	wetterSensor[0][sensorImArray][1] = timer1Sek;
	wetterSensor[0][sensorImArray][2] = (double)wert[1] * 500 / 1023;
	wetterSensor[0][sensorImArray][3] = (double)wert[0] * 500 / 3069;

	return 1;
}

void verlaufArrayVorschieben() {
  int aktuelleZeit = timer1Sek;
  if (((aktuelleZeit - letzteVerschiebungszeitpunkt) < abstandHistorie)) {
    return;
  }

  for (int i = historischeWerteAnzahl-1; i >= 0; i--) {
    for (int j = 0; j < sensorAnzahl; j++) {
      for (uint8_t k = 0; k < 4; k++) {
        wetterSensor[i + 1][j][k] = wetterSensor[i][j][k];
      }
      #ifdef DEBUG
          // Anzeige auf Seriellen Monitor
          Serial.print("\n historischer Verlauf weitergeschoben");   
          Serial.print("   ");
          for (uint8_t k = 0; k < historischeWerteAnzahl; k++) {
            Serial.print(k);
            Serial.print(": ");
            Serial.print(wetterSensor[k][j][1]);
            Serial.print(", ");
            Serial.print(wetterSensor[k][j][2]);
            Serial.print(", ");
            Serial.print(wetterSensor[k][j][3]);
            Serial.print(", ");
          }
       
          Serial.println();
      #endif  
    }
  }

  letzteVerschiebungszeitpunkt = aktuelleZeit;
  return;
}

void sensorNameFestlegen() {
	sensorBezeichnung = "Default";
	switch (anzeigeSensor) {
	case 0:
		sensorBezeichnung = "Flur";
		break;
	case 1:
		sensorBezeichnung = "aussen";
		break;
	case 2:
		sensorBezeichnung = "Keller";
		break;
	default:
		sensorBezeichnung = "Testmod.";
		break;
	}
}

void aktuelleWerteAnzeigen(int displayBreite, int displayHoehe) {
	sensorNameFestlegen();

	if (displayHoehe < 1 || displayBreite < 15) {
#ifdef DEBUG
		Serial.print("\n Das definierte Display ist zu klein.");
#endif  
		return;
	}
	lcd.clear();
	lcd.setCursor(0, displayHoehe - 1);
	lcd.write(thermometerChar);
	lcd.print(wetterSensor[0][anzeigeSensor][2]);
	lcd.write(gradChar);
	lcd.write(celsiusChar);
	lcd.setCursor(displayBreite - 7, displayHoehe - 1);
	lcd.write(tropfenChar);
	lcd.print(wetterSensor[0][anzeigeSensor][3]);
	lcd.write(prozentChar);

	if (displayHoehe > 1) {
		lcd.setCursor(0, 0);
		lcd.print(sensorBezeichnung);
	}
}


void prozentBalkenZeigen(double geanderterWertProzentual, int maxCharZeichen) {
	sensorNameFestlegen();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(sensorBezeichnung);
	lcd.setCursor(maxCharZeichen - 8, 0);
	lcd.write(deltaChar);
	lcd.write(tropfenChar);
	lcd.print(geanderterWertProzentual);
	lcd.write(prozentChar);

	int xKoordinate = maxCharZeichen * 0.5;
	int yKoordinate = 1;
	double prozentInGanzeKaestchen = maxCharZeichen / 100.0 * geanderterWertProzentual* 0.5;

	// positive Wertänderung verarbeiten
	if (prozentInGanzeKaestchen > 0) {
		for (int i = 0; i < prozentInGanzeKaestchen; i++) {
			lcd.setCursor(xKoordinate - 1 + i, yKoordinate);
			lcd.write(255);
		}

		//Änderungspfeil anzeigen
		lcd.setCursor(xKoordinate + 1 + prozentInGanzeKaestchen, yKoordinate);
		lcd.write(126);
	}

	// negative Wertänderung verarbeiten
	if (prozentInGanzeKaestchen < 0) {
		for (int i = 0; i > prozentInGanzeKaestchen; i--) {
			lcd.setCursor(xKoordinate - 1 + i, yKoordinate);
			lcd.write(255);
		}

		//Änderungspfeil anzeigen
		lcd.setCursor(xKoordinate - 1 + prozentInGanzeKaestchen, yKoordinate);
		lcd.write(127);
	}
}

//LED schneller blinken lassen zwischen 60% und 100% Luftfeuchte
void pruefungFeuchtigkeitUeber60() {
	double maxFeuchtigkeit = 0;

	for (int8_t sensorAbfragen = 0; sensorAbfragen < sensorAnzahl + 1; sensorAbfragen++) {
		if (wetterSensor[0][sensorAbfragen][3] > maxFeuchtigkeit) {
			maxFeuchtigkeit = wetterSensor[0][sensorAbfragen][3];
		}
	}
	if (maxFeuchtigkeit >= 60) {
		maxFeuchtigkeit = (100 - maxFeuchtigkeit) * 3 / 8;
		if (maxFeuchtigkeit < 1) {
			maxFeuchtigkeit = 1;
		}
		blinkzeitLedRt = maxFeuchtigkeit;
		return;
	}
	blinkzeitLedRt = 0;
	PORTB &= ~(1 << ledrtPD); //LED ausschalten

	return;
}

void testWetterStation() {
	if (wetterSensor[0][sensorAnzahl][3] <= 0) { //Feuchtigkeit = 0 (Beginn des Testmodus) - 1. Werte setzen
		wetterSensor[0][sensorAnzahl][2] = -5;
		wetterSensor[0][sensorAnzahl][3] = 55;
		letzteAbrufzeit[sensorAnzahl][1] = timer1Sek;
		anzeigeSensor = sensorAnzahl;
		return;
	}
	if ((timer1Sek - letzteAbrufzeit[sensorAnzahl][1]) > 4) {
		letzteAbrufzeit[sensorAnzahl][1] = timer1Sek;
		wetterSensor[0][sensorAnzahl][2] = wetterSensor[0][sensorAnzahl][2] + 5;
		wetterSensor[0][sensorAnzahl][3] = wetterSensor[0][sensorAnzahl][3] + 5;
		pruefungFeuchtigkeitUeber60;
		aktuelleWerteAnzeigen(lcdBreite, lcdHoehe);
		if (wetterSensor[0][sensorAnzahl][3] > 100) {	//Ende der Testroutine
			wetterSensor[0][sensorAnzahl][2] = 0;
			wetterSensor[0][sensorAnzahl][3] = 0;
			tasterBetaetigung = 0;
			anzeigeSensor = 0;
			PORTB &= ~(1 << ledgePD);	//LEDge ausschalten
			aktuelleWerteAnzeigen(lcdBreite, lcdHoehe);
		}
	}
	return;
}
