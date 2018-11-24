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
#define abrufIntervallSekunden 5
#define abstandHistorie 60

// GLOBALE VARIABLEN
const uint8_t historischeWerteAnzahl = 10;
const uint8_t sensorAnzahl = 3;
//Array fuer Temp und Feuchtigkeit, v.l.n.r.: Historie- 3.Dimension,  Sensor-Nr.(PORT)- 2. Dimension, Sensor-Werte- 1. Dimension
//Werte 1.Dimesion: 0: Nr. des Sensors, 1: Messzeitpunkt der Messung (ab Start vom Arduino in s), 2.Wert: Feucht, 3.Wert: Temp
double wetterSensor[historischeWerteAnzahl][sensorAnzahl][4];

uint32_t letzteAbrufzeit[sensorAnzahl][2]; //0. Stelle: letzter Abfragewert Sensor, 1. Stelle: letzte Verschiebung historischer Daten

unsigned int eepromMaximum = 0;

uint8_t interruptPinD = 0;

volatile uint8_t timer1_over = 0;
volatile uint8_t timer2_over = 0;
volatile uint8_t portInterrupt = 0;

uint8_t timerTaster = 0;
uint8_t blinkzeitLed = 1;	//zwischen 0 und 12, 0-> gar nicht blinken; 12 (ca.7s leuchten)- langsames blinken; 1-schnelles blinken

uint8_t tasterVorher = 0;
uint8_t tasterGedruckt = 0;	//0: nicht gedruckt, 1: gedrueckt, 2: lang gedruckt7
uint8_t anzeigeSensor = 0;	//welcher Sensor soll angezeigt werden? (0 -> 0.Wert im Array)


//eigene Symbole
char thermometerChar = 0;
char feuchtigkeitChar = 0;
byte thermometerByte[8] = {
  B01110,
  B01010,
  B01010,
  B01010,
  B11011,
  B10001,
  B11011,
  B01110
};
byte feuchtigkeitByte[8] = {
  B00000,
  B00100,
  B00100,
  B01110,
  B01110,
  B11111,
  B01110,
  B00100
};

// PORTDEKLARATIONEN
#define ledrtPD	PB5			//PIN fuer rote LED
#define ledgePD	PB4			//PIN fuer geldbe 
#define taster  PB0			//PIN fuer Taster zum Umschalten der Sensoren (PIN8)
#define feuchtLuftPD  PD3	//PIN fuer den Feuchtigkeits- und Temp- Sensor (PIN3)
#define dht22Sensor   PD4	//PIN fuer DHT22-Sensor (Feuchtigkeit+Temp)



// ######################## INTERRUPT SERVICE ROUTINEN ###############################

// Routine zum Abfragen der Taster
ISR(PCINT0_vect) {
	uint8_t aktuellerTasterWert;

	// wurde Taster gedrueckt?
	aktuellerTasterWert = (PINB & (1 << taster) >> taster);
	if (aktuellerTasterWert == 0) {	//wurde taster angesprochen?
		// entprellen
		if (tasterVorher == 0) {	// Taster wurde zum ersten Mal gedrueckt
			tasterVorher = 1;
			timerTaster = 0;
			return;
		}
	}
}

// Abfragen des Sensors am Port D3 (DHT11)
ISR(INT1_vect)
{
	portInterrupt = 3;
}

 // Routine zum Abfragen von Sensoren nicht am Port D2 und D3
ISR(PCINT2_vect) { //Sensoren an PORT D, die nicht an D2 und D3 haengen
	interruptPinD++;
	portInterrupt = 4;
	if ((PIND & (1 >> PCINT20) << PCINT20) > 0) {
		portInterrupt = 4;
	}
}


// Timer1 Taster und rote LED (30ms)
ISR(TIMER1_COMPA_vect) {
	if (tasterVorher > 0) {
		// 50 ms nach Tastendruck warten, um Prellen auszuschließen
		if (timerTaster > 0) {
			PORTB ^= (1 << ledgePD);  // LED toggelnd
			tasterGedruckt = 1;
			tasterVorher = 0;
		}
	}
	if (blinkzeitLed > 0) {
		if (timer1_over / 20 == blinkzeitLed) {
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
  lcd.createChar(feuchtigkeitChar, feuchtigkeitByte);
  lcd.createChar(thermometerChar, thermometerByte);
  #ifdef DEBUG
  	// Ausgabe am Monitor vorbereiten (Debug)
  	Serial.begin(9600);
  	Serial.println("\n\nAusgabe am Monitor");
  	Serial.println("------------------");
  	Serial.print("eepromMaximum = ");
  	Serial.println(eepromMaximum);
  	Serial.println("------------------");
  #endif

	// Deklarieren der I-O-Ports und Setzen der Pegel
	DDRB |= (1 << ledrtPD);     // als Ausgang
	PORTB |= (1 << ledrtPD);	//HIGH setzen
	DDRB |= (1 << ledgePD);		// als Ausgang
	PORTB |= (1 << ledgePD);	//HIGH setzen

	DDRB &= ~(1 << taster); // als Eingang setzen - LOW-aktiv 
	PORTB |= (1 << taster); // Pull-Up-Widerstand am Arduino setzen
	DDRD |= (1 << feuchtLuftPD);  // als Ausgang setzen
	DDRD |= (1 << dht22Sensor); // als Ausgang setzen

	// Variablen zuruecksetzen
	// leere Array wetterSensor und fülle mit Sensor-Nummer
	for (uint8_t i = 0; i < 6; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			for (uint8_t k = 0; k < 4; k++) {
				wetterSensor[i][j][k] = 0;
			}
		}
		wetterSensor[i][0][0] = feuchtLuftPD;	//DHT11 an Port D3
		wetterSensor[i][1][0] = dht22Sensor;	//DHT22 an Port D4
		wetterSensor[i][2][0] = 23;  //Analoger EIngang 23 (+24) fuer analogen Sensor (A0+A1)
	}

	// leere Array letzte Zugriffszeit und setze Sensor-Nr.
	for (uint8_t i = 0; i < 3; i++) {
		for(uint8_t j =0; j<2;j++){
			letzteAbrufzeit[i][j] = 0;
		}
	}

	// ()()()()()())()()()()() INTERRUPTS ()()()()()()()()()()()()()()()()()()()()
	cli();	//deaktivieren von Interrupts

	// TIMER1 fuer Taster und LED
	TCCR1A = 0; // TCCR1A register auf 0 setzen
	TCCR1B = 0; // TCCR1B register auf 0 setzen
	TCNT1 = 0; // Zaehlerwert zuruecksetzen
	OCR1A = 12499;	//MAX bei 16-BIT-TIMER: 65535, bei 8-Bit 2 hoch 8 - 1 = 255
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

	// Interrupt auf PORTD-Gruppe (Sensoren)
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT20);

	// Interrupt auf PORT PD3 (Sensor DHT11)
	EICRA |= (1 << ISC10); //fallende und steigende Flanke erzeugt einen Interrupt
	EIMSK |= (1 << INT1);

	sei();	//aktivieren von Interrupts
}

// ########################## HAUPTPROGRAMM (LOOP) ######################################
void loop() {
	int8_t kontrolle = 0;
	kontrolle = sensorFeuchtTempAbfrage(feuchtLuftPD);
	
	// Events bei neuen Sensorwerten
	if (kontrolle > 0) {
    prozentBalkenZeigen("sarrus", -20, -30, 16);
		aktuelleWerteAnzeigen();
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
			Serial.println(letzteAbrufzeit[i][0]);
			Serial.print("Kontrolle: ");
			Serial.println(kontrolle);
		}
	}
	interruptPinD = 0;
#endif
	kontrolle = sensorFeuchtTempAbfrage(dht22Sensor);

#ifdef DEBUG
	if (kontrolle != -99) {
		Serial.print("Kontrolle: ");
		Serial.println(kontrolle);
		Serial.print("Interrupt: ");
		Serial.println(interruptPinD);
	}
#endif // DEBUG


	// Events bei neuen Sensorwerten
	if (kontrolle > 0) {
    prozentBalkenZeigen("sarrus", -20, -30, 16);
		aktuelleWerteAnzeigen();
	}
	// Events bei Tastendruck
	if (tasterGedruckt > 0) {
		tasterGedruckt = 0;
		anzeigeSensor = (anzeigeSensor + 1) % 3;
		aktuelleWerteAnzeigen();
    #ifdef DEBUG
    		Serial.print("AnzeigeSensor: ");
    		Serial.println(anzeigeSensor);
    #endif
	}
}

// ############## ABFRAGEN DES DIGITALEN SENSORS ############################
int sensorFeuchtTempAbfrage(uint8_t pin) {
	//Variablen
	int8_t sensorImArray = -1;
	int8_t kontrolle = 0;
	int8_t i = 0, j = 0;	//Zaehlervariablen
	uint8_t wert[5];    //Bytes des Sensorwertes
	uint8_t bitwert = 7;  //Bit der einzelnen Bytes des Sensorwertes
	uint16_t zaehler;
	uint16_t sum = 0;	//Zeit fuer Datenbit, Paritaetssumme

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
	if (((millis() / 1000) - letzteAbrufzeit[sensorImArray][0]) < abrufIntervallSekunden) {
		if (letzteAbrufzeit[sensorImArray][0] != 0) {// nach Einschalten soll gemessen werden
			return -99;
		}
	}
	letzteAbrufzeit[sensorImArray][0] = (millis() / 1000);

	// BUFFER leeren
	for (i = 0; i < 5; i++) {
		wert[i] = 0;
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
			return -11;
		}
	}
	TCNT2 = 0;
	PORTD |= (1 << pin);	// auf HIGH setzen
	timer2_over = 0;
	zaehler = MAXZAEHL;
	//1 count = 2 Mikro-Sekunden -> 40 Mikrosekunden
	while (TCNT2 < 22) {
		if (zaehler-- <= 0) {
			return -11;
		}
	}
	DDRD &= ~(1 << pin);	// PIN als Eingang setzen

	// ------------------------- Antwort vom Sensor: 1. Bit ist LOW, 2. Bit ist HIGH, 3. Bit LOW: PAUSE -------------------------------
	for (i = 2; i < 4; i++) {
		TCNT2 = 0; // setzte timer 0 zurueck
		timer2_over = 0;
		portInterrupt = 0;
		zaehler = MAXZAEHL;
		//Warte 4us
		while (TCNT2 <= 1) {
			if (zaehler-- <= 0) {
				kontrolle = -29-i;
				return kontrolle;
			}
		}
		do {
			if (zaehler-- <= 0) {
				kontrolle = -20-i;
				return kontrolle;
			}
		} while (portInterrupt == 0);
	}


	//----------------------------------- ab jetzt kommen die Daten ---------------------------------------
	// 5 BYTES Daten
	for (i = 0; i < 5; ++i) {
		// 8 Bits pro Byte Daten (beginn mit MSB)
		for (j = 7; j >= 0; j--) {
			//PAUSE, danach DATEN:
			for (int k = 5; k < 7; k++) {
				portInterrupt = 0;
				zaehler = MAXZAEHL;
				TCNT2 = 0; // setzte timer 0 zurueck
				timer2_over = 0;
				do {
					if (zaehler-- <= 0) {
						kontrolle = -30-k;
						return kontrolle;
					}
				} while (portInterrupt == 0);
				portInterrupt = 0;
			}
			sum = TCNT2;
			portInterrupt = 0;

			//gesendetes Bit HIGH?
			if (sum >= 20) {	//2us pro hochgezaehlten Bit
				wert[i] |= (1 << j);
			}
		}
	}
	DDRD |= (1 << pin);  // als Ausgang setzen
	//Paritaetspruefung
	sum = wert[0] + wert[1] + wert[2] + wert[3];

#ifdef DEBUG
	Serial.println("\n#############################");
	Serial.print("Sensor ");
	Serial.println(sensorImArray);
	if (sum != wert[4]) {
		Serial.println("Paritaetspruefung fehlgeschlagen");
	}
	// Anzeige zur Fehlereingrenzung)
	Serial.print("Fehler: ");
	Serial.print(kontrolle);

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
#endif

	if (sum != wert[4]) {
		return -50;
	}

	if (kontrolle <0) {
		return kontrolle;
	}

	// Pruefung auf Plausibilitaet der Daten ->-> muss noch gemaccht werden

	//Werte schreiben in Array
	wetterSensor[0][sensorImArray][1] = millis() / 1000;
	wetterSensor[0][sensorImArray][2] = wert[3];
	wetterSensor[0][sensorImArray][2] = wetterSensor[0][sensorImArray][2] / 10 + wert[2];
	wetterSensor[0][sensorImArray][3] = wert[1];
	wetterSensor[0][sensorImArray][3] = wetterSensor[0][sensorImArray][3] / 10 + wert[0];

	letzteAbrufzeit[sensorImArray][0] = wetterSensor[0][sensorImArray][1];
	return 1;
}

void verlaufArrayVorschieben() {
	for (uint8_t historieEbene = 0; historieEbene < historischeWerteAnzahl; historieEbene++) {
		for (uint8_t einzelnerSensor = 0; einzelnerSensor < 3; einzelnerSensor++) {
			if ((((millis() / 1000) - letzteAbrufzeit[einzelnerSensor][1]) < abstandHistorie) 
				|| (letzteAbrufzeit[einzelnerSensor][0] > wetterSensor[0][einzelnerSensor][1])) {
				return;
			}
			for (uint8_t werteSensor = 0; werteSensor < 4; werteSensor++) {
				wetterSensor[historieEbene + 1][einzelnerSensor][werteSensor] = wetterSensor[historieEbene][einzelnerSensor][werteSensor];
			}
			letzteAbrufzeit[einzelnerSensor][1] = (millis() / 1000);
#ifdef DEBUG
			// Anzeige auf Seriellen Monitor
			Serial.print("\n historischer Verlauf weitergeschoben, Sensor: ");
			Serial.println(wetterSensor[0][einzelnerSensor][0]);
			Serial.print("Daten des 1. Historischen Wertes: ");
			for (uint8_t werteSensor = 0; werteSensor < 4; werteSensor++) {
				Serial.print(wetterSensor[1][einzelnerSensor][werteSensor]);
				Serial.print(", ");
			}
			Serial.println();
#endif	
		}
	}
}

void aktuelleWerteAnzeigen() {
	lcd.clear();

	//Werte auf LCD drucken
	lcd.setCursor(0, 0);
	//lcd.print("Luftf.: ");
  lcd.write(thermometerChar);
	lcd.setCursor(10, 0);
	lcd.print(wetterSensor[0][anzeigeSensor][3]);
	lcd.setCursor(0, 1);
	//lcd.print("Temp.: ");
  lcd.write(feuchtigkeitChar);
	lcd.setCursor(10, 1);
	lcd.print(wetterSensor[0][anzeigeSensor][2]);
}


void prozentBalkenZeigen (String bezeichnung, double geanderterWertAbsolut, double geanderterWertProzentual, int maxCharZeichen)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(bezeichnung);
  lcd.setCursor(10, 0);
  lcd.print(geanderterWertAbsolut);
  
  int xKoordinate = maxCharZeichen * 0.5;
  int yKoordinate = 1;
  double prozentInGanzeKaestchen = maxCharZeichen / 99.9 * geanderterWertProzentual* 0.5;

  // positive Wertänderung verarbeiten
  if (prozentInGanzeKaestchen > 0)
  {
    for (int i = 0; i < prozentInGanzeKaestchen; i++)
    {
      lcd.setCursor(xKoordinate - 1 + i, yKoordinate);
      lcd.write(255);
    }
  
  //Änderungspfeil anzeigen
  lcd.setCursor(xKoordinate + 1 + prozentInGanzeKaestchen, yKoordinate);
  lcd.write(126);
  }
  
  // negative Wertänderung verarbeiten
  if (prozentInGanzeKaestchen < 0)
  {
    for (int i = 0; i > prozentInGanzeKaestchen; i--)
    {
      lcd.setCursor(xKoordinate - 1 + i, yKoordinate);
      lcd.write(255);
    }
  
  //Änderungspfeil anzeigen
  lcd.setCursor(xKoordinate - 1 + prozentInGanzeKaestchen, yKoordinate);
  lcd.write(127);
  }
}
